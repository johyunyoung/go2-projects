"""Shared helpers: load unitree_cpp_deploy CSV logs, rebuild observations, actuator model fits, MuJoCo replay.

CSV columns (unitree_cpp_deploy State_RLBase logger): time, unix_time, wall_time, q_des_0..11 (POLICY joint order),
q/dq/tau/temp_0..11 (SDK motor order), imu_rpy, imu_acc, ang_vel, foot_force (SDK foot order FR,FL,RR,RL),
foot_contact, cmd_ns_0..2 (raw joystick ly,-lx,-rx), cmd_fixed_0..2.
"""
import os
import numpy as np
import pandas as pd

MAP = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])   # policy joint i <- SDK motor MAP[i] (deploy.yaml joint_ids_map)
DEFAULT = np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5])
JN = [f"{l}_{j}" for l in ["FL", "FR", "RL", "RR"] for j in ["hip", "thigh", "calf"]]
G = {"hip": [0, 3, 6, 9], "thigh": [1, 4, 7, 10], "calf": [2, 5, 8, 11]}
RANGES = np.array([[-2.0, 2.0], [-1.0, 1.0], [-2.0, 2.0]])  # deploy.yaml command ranges
KP, KD = 25.0, 0.5


# ---------------------------------------------------------------- loading
def load(log_dir, name, kind="velocity"):
    """log_dir has velocity/ and stand/ sub-folders with run_data_<name>.csv. Parquet cache in log_dir/_cache."""
    cache = os.path.join(log_dir, "_cache")
    os.makedirs(cache, exist_ok=True)
    pq = os.path.join(cache, f"{kind}_{name}.parquet")
    if os.path.exists(pq):
        return pd.read_parquet(pq)
    df = pd.read_csv(os.path.join(log_dir, kind, f"run_data_{name}.csv")).dropna()  # last row may be truncated
    df.to_parquet(pq)
    return df


def arrays(df):
    def g(p, n):
        return df[[f"{p}_{i}" for i in range(n)]].to_numpy(float)
    return dict(t=df["time"].to_numpy(float), wall=df["wall_time"].to_numpy(),
                q_des=g("q_des", 12), q=g("q", 12)[:, MAP], dq=g("dq", 12)[:, MAP], tau=g("tau", 12)[:, MAP],
                temp=g("temp", 12)[:, MAP], rpy=g("imu_rpy", 3), acc=g("imu_acc", 3), gyro=g("ang_vel", 3),
                foot=g("foot_force", 4), joy=g("cmd_ns", 3), fixed=g("cmd_fixed", 3))


def segments(t, gap=0.05):
    """Contiguous Velocity-state segments (the logger only writes while the policy runs)."""
    br = np.where(np.diff(t) > gap)[0]
    return list(zip(np.r_[0, br + 1], np.r_[br + 1, len(t)]))


def command(joy, fixed=None):
    """unitree_cpp_deploy VelocityCommandDamper: joystick x range max (NOT clamp). Full stick = 2 m/s."""
    c = np.where(joy > 0, joy * RANGES[:, 1], joy * -RANGES[:, 0])
    if fixed is not None:
        on = np.abs(fixed).max(1) > 0
        c[on] = fixed[on]
    return c


def gravity_b(rpy):
    r, p = rpy[:, 0], rpy[:, 1]
    return np.stack([np.sin(p), -np.cos(p) * np.sin(r), -np.cos(p) * np.cos(r)], 1)


# ---------------------------------------------------------------- policy replay
def policy_steps(q_des):
    return np.where(np.any(np.diff(q_des, axis=0) != 0, axis=1))[0] + 1


def replay(a, s, e, onnx_path, H=10):
    """Rebuild the 450-d observation (per-term blocks, oldest->newest inside a block) and run policy.onnx."""
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path)
    name = sess.get_inputs()[0].name
    st = policy_steps(a["q_des"][s:e]) + s
    ob = st - 1                                   # state logged just before the new target appears
    act = (a["q_des"][st] - DEFAULT) / 0.25
    feats = [a["gyro"][ob] * 0.25, gravity_b(a["rpy"][ob]), command(a["joy"][ob], a["fixed"][ob]),
             a["q"][ob] - DEFAULT, a["dq"][ob] * 0.05,
             np.vstack([(a["q_des"][st[0] - 1] - DEFAULT) / 0.25, act[:-1]])]
    X = np.array([np.concatenate([f[k - H + 1:k + 1].reshape(-1) for f in feats]) for k in range(H - 1, len(st))],
                 np.float32)
    pred = np.vstack([sess.run(None, {name: x[None]})[0] for x in X])
    return act[H - 1:], pred


# ---------------------------------------------------------------- actuator fit
def fit_actuator(a, mask, delays):
    """tau_est ~ g_kp*KP*(q_des(t-delay)-q) - g_kd*KD*dq + c*sign(dq) + b, per joint. Returns R2[delay,joint], coef."""
    t, q, dq, tau, qd = a["t"], a["q"], a["dq"], a["tau"], a["q_des"]
    R2 = np.zeros((len(delays), 12))
    C = np.zeros((len(delays), 12, 4))
    for k, d in enumerate(delays):
        i = np.searchsorted(t, t - d, side="right") - 1
        m = mask & (i >= 0)
        qs = qd[np.clip(i, 0, None)]
        for j in range(12):
            A = np.stack([KP * (qs[m, j] - q[m, j]), -KD * dq[m, j], np.sign(dq[m, j]), np.ones(m.sum())], 1)
            c, *_ = np.linalg.lstsq(A, tau[m, j], rcond=None)
            r = tau[m, j] - A @ c
            R2[k, j] = 1 - r.var() / tau[m, j].var()
            C[k, j] = c
    return R2, C


def unitree_curve_exceeded(dq, tau, Y1=20.2, Y2=23.4, X1=13.5, X2=30.0, tol=0.5):
    """True where |tau| is above the training (Go2HV) torque-speed curve."""
    lim = np.where(dq * tau > 0, Y1, Y2)
    v = np.abs(dq)
    lim = np.where(v < X1, lim, np.clip(lim - lim / (X2 - X1) * (v - X1), 0, None))
    return np.abs(tau) > lim + tol


# ---------------------------------------------------------------- terrain-ish window classes
def window_class(t, rpy, win=2.0):
    """0 calm-level (hard flat), 1 rough (grass etc.), 2 tilted (stairs/slope); -1 unclassified."""
    fs = 1 / np.median(np.diff(t))
    n = int(win * fs)
    cls = np.full(len(t), -1)
    for w0 in range(0, len(t) - n, n):
        w = slice(w0, w0 + n)
        if np.ptp(t[w]) > win * 1.1:
            continue
        p, r = np.degrees(rpy[w, 1]), np.degrees(rpy[w, 0])
        tilt = max(abs(p.mean()), abs(r.mean()))
        cls[w] = 0 if (p.std() < 2.0 and r.std() < 3.0 and tilt < 5) else (2 if tilt > 10 else 1)
    return cls


SPEED_BINS = [(0.3, 0.8, "slow"), (0.8, 1.4, "mid"), (1.4, 2.1, "fast")]


def gait_freq(t, thigh, cmdx, mask, lo, hi):
    fs = 1 / np.median(np.diff(t))
    n = int(4 * fs)
    fr = []
    for w0 in range(0, len(t) - n, n // 2):
        w = slice(w0, w0 + n)
        if np.ptp(t[w]) > 4.2 or not (mask[w] & (np.abs(cmdx[w]) >= lo) & (np.abs(cmdx[w]) < hi)).all():
            continue
        x = thigh[w] - thigh[w].mean()
        F = np.abs(np.fft.rfft(x * np.hanning(n)))
        f = np.fft.rfftfreq(n, 1 / fs)
        b = (f > 0.8) & (f < 6)
        fr.append(f[b][F[b].argmax()])
    return float(np.median(fr)) if fr else float("nan")


def stats(t, qd, q, dq, tau, rpy, gyro, cmdx, mask):
    out = {}
    for lo, hi, nm in SPEED_BINS:
        m = mask & (np.abs(cmdx) >= lo) & (np.abs(cmdx) < hi)
        if m.sum() < 2000:
            continue
        out[nm] = dict(
            seconds=float(m.sum() * np.median(np.diff(t))),
            gait_hz=gait_freq(t, q[:, 1], cmdx, mask, lo, hi),
            gyro_rms=float(np.sqrt((gyro[m] ** 2).mean())),
            pitch_std_deg=float(np.degrees(rpy[m, 1].std())),
            calf_dq_p99=float(np.percentile(np.abs(dq[m][:, G["calf"]]), 99)),
            thigh_tau_p99=float(np.percentile(np.abs(tau[m][:, G["thigh"]]), 99)),
            calf_tau_p99=float(np.percentile(np.abs(tau[m][:, G["calf"]]), 99)),
            power_p99_w=float(np.percentile(np.abs(tau[m] * dq[m]).sum(1), 99)),
            calf_track_rms=float(np.sqrt(((qd[m][:, G["calf"]] - q[m][:, G["calf"]]) ** 2).mean())),
            calf_over_train_curve_pct=float(100 * unitree_curve_exceeded(dq[m][:, G["calf"]], tau[m][:, G["calf"]]).mean()))
    return out


# ---------------------------------------------------------------- MuJoCo replay of a command sequence
ACT = {"isaac": dict(hip=(20.2, 23.4, 13.5, 30), thigh=(20.2, 23.4, 13.5, 30), calf=(20.2, 23.4, 13.5, 30)),
       "sim2real": dict(hip=(20.2, 23.4, 13.5, 30), thigh=(20.2, 23.4, 13.5, 30), calf=(30.3, 35.1, 9.0, 30))}


def sim_replay(xml, onnx_path, cmds, act="isaac", delay_steps=1, DT=0.005, DEC=4, H=10):
    """Flat-ground replay with the training convention (kp25/kd0.5, 0.005 s x 4, Unitree torque-speed clip).
    Returns rows: t, q_des(12), q(12), dq(12), tau(12), rpy(3), gyro(3), cmd(3), base lin vel world(3), base pos(3)."""
    import mujoco
    import onnxruntime as ort
    m = mujoco.MjModel.from_xml_path(xml)
    m.opt.timestep = DT
    m.dof_frictionloss[6:] = 0.0                  # losses handled by the explicit torque model
    d = mujoco.MjData(m)
    d.qpos[2] = 0.36
    d.qpos[3:7] = [1, 0, 0, 0]
    d.qpos[7:19] = DEFAULT
    sess = ort.InferenceSession(onnx_path)
    nm = sess.get_inputs()[0].name
    lim = ACT[act]
    grp = ["hip", "thigh", "calf"] * 4
    Y1, Y2, X1, X2 = (np.array([lim[g][i] for g in grp]) for i in range(4))

    def clip(dq, tau):
        l = np.where(dq * tau > 0, Y1, Y2)
        v = np.abs(dq)
        l = np.where(v < X1, l, np.clip(l - l / (X2 - X1) * (v - X1), 0, None))
        return np.clip(tau, -l, l)

    for _ in range(200):                          # settle 1 s at the default pose
        d.qfrc_applied[6:18] = clip(d.qvel[6:18], KP * (DEFAULT - d.qpos[7:19]) - KD * d.qvel[6:18])
        mujoco.mj_step(m, d)
    last = (d.qpos[7:19] - DEFAULT) / 0.25
    pipe = [DEFAULT.copy()] * (delay_steps + 1)
    hist = None
    out = []
    for k, cmd in enumerate(cmds):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, d.qpos[3:7])
        R = R.reshape(3, 3)
        f = [d.qvel[3:6] * 0.25, R.T @ np.array([0, 0, -1.0]), cmd, d.qpos[7:19] - DEFAULT, d.qvel[6:18] * 0.05, last]
        if hist is None:
            hist = [[np.array(x, float) for _ in range(H)] for x in f]
        for i, x in enumerate(f):
            hist[i] = hist[i][1:] + [np.array(x, float)]
        obs = np.concatenate([np.concatenate(h) for h in hist]).astype(np.float32)[None]
        a = sess.run(None, {nm: obs})[0][0]
        last = a.copy()
        q_des = DEFAULT + 0.25 * a
        rpy = np.array([np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(np.clip(R[2, 0], -1, 1)), 0])
        for s in range(DEC):
            pipe.append(q_des.copy())
            qd = pipe.pop(0)
            tau = clip(d.qvel[6:18], KP * (qd - d.qpos[7:19]) - KD * d.qvel[6:18])
            d.qfrc_applied[6:18] = tau
            mujoco.mj_step(m, d)
            out.append(np.concatenate([[(k * DEC + s) * DT], qd, d.qpos[7:19], d.qvel[6:18], tau, rpy,
                                       d.qvel[3:6], cmd, d.qvel[0:3], d.qpos[0:3]]))
        if d.qpos[2] < 0.12:
            print("sim: fell at policy step", k)
            break
    return np.array(out)
