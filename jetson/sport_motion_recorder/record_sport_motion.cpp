// Record Go2 sport-mode stand-up / sit-down motions (READ ONLY: never sends a motor command).
//
// Subscribes rt/lowstate (joints, IMU, feet, remote) and rt/sportmodestate (sport mode, body height) and
// writes one CSV row per lowstate message (~500 Hz). Used to replay the original Unitree StandUp/StandDown
// trajectories in our RL controller (go2-projects jetson/sport_motion_recorder/README.md).
//
//   manual: ./record_sport_motion -n eth0 -o ~/go2_ws/logs/sport_motion           (you press L2+A on the remote; Ctrl+C to stop)
//   auto  : ./record_sport_motion -n eth0 -o ~/go2_ws/logs/sport_motion --auto 3  (calls SportClient StandUp/StandDown 3 times)
//
// Sport mode must be the active controller (the RL controller / go2-rl service must be stopped).

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using namespace unitree::robot;
using LowState = unitree_go::msg::dds_::LowState_;
using SportState = unitree_go::msg::dds_::SportModeState_;

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run = false; }

struct Recorder {
    FILE* f = nullptr;
    std::mutex mtx;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::atomic<uint64_t> rows{0};
    // latest sport mode state (written by its own callback)
    std::atomic<int> sport_mode{-1}, gait{-1};
    std::atomic<float> progress{0}, body_height{0}, pos_z{0};
    std::string event;  // annotated on the next row
    std::atomic<float> last_calf{0};

    void header() {
        // SDK motor order: FR(hip,thigh,calf) FL RR RL
        const char* leg[4] = {"FR", "FL", "RR", "RL"};
        const char* jn[3] = {"hip", "thigh", "calf"};
        std::fprintf(f, "t,tick");
        for (const char* k : {"q", "dq", "tau", "temp"})
            for (int l = 0; l < 4; ++l) for (int j = 0; j < 3; ++j) std::fprintf(f, ",%s_%s_%s", k, leg[l], jn[j]);
        std::fprintf(f, ",quat_w,quat_x,quat_y,quat_z,roll,pitch,yaw,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z");
        std::fprintf(f, ",foot_FR,foot_FL,foot_RR,foot_RL,remote_keys,sport_mode,sport_gait,sport_progress,body_height,pos_z,event\n");
    }

    void on_lowstate(const void* msg) {
        const auto& s = *static_cast<const LowState*>(msg);
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::lock_guard<std::mutex> lk(mtx);
        if (!f) return;
        std::fprintf(f, "%.4f,%u", t, s.tick());
        for (int i = 0; i < 12; ++i) std::fprintf(f, ",%.5f", s.motor_state()[i].q());
        for (int i = 0; i < 12; ++i) std::fprintf(f, ",%.4f", s.motor_state()[i].dq());
        for (int i = 0; i < 12; ++i) std::fprintf(f, ",%.3f", s.motor_state()[i].tau_est());
        for (int i = 0; i < 12; ++i) std::fprintf(f, ",%d", (int)s.motor_state()[i].temperature());
        const auto& imu = s.imu_state();
        std::fprintf(f, ",%.5f,%.5f,%.5f,%.5f", imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2], imu.quaternion()[3]);
        std::fprintf(f, ",%.5f,%.5f,%.5f", imu.rpy()[0], imu.rpy()[1], imu.rpy()[2]);
        std::fprintf(f, ",%.4f,%.4f,%.4f,%.3f,%.3f,%.3f", imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2],
                     imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2]);
        for (int i = 0; i < 4; ++i) std::fprintf(f, ",%d", (int)s.foot_force()[i]);
        const auto& wr = s.wireless_remote();
        const unsigned keys = (unsigned)wr[2] | ((unsigned)wr[3] << 8);   // button bit field (L2, A, start, ...)
        std::fprintf(f, ",%u,%d,%d,%.3f,%.4f,%.4f,%s\n", keys, sport_mode.load(), gait.load(), progress.load(),
                     body_height.load(), pos_z.load(), event.c_str());
        event.clear();
        last_calf = s.motor_state()[2].q();
        rows++;
    }

    void on_sport(const void* msg) {
        const auto& s = *static_cast<const SportState*>(msg);
        sport_mode = s.mode(); gait = s.gait_type(); progress = s.progress();
        body_height = s.body_height(); pos_z = s.position()[2];
    }

    void mark(const std::string& e) { std::lock_guard<std::mutex> lk(mtx); event = e; std::cout << "[event] " << e << std::endl; }
};

static void sleep_while_running(double s) {
    auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(s);
    while (g_run && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

int main(int argc, char** argv) {
    std::string net = "eth0", out = "./sport_motion";
    int auto_cycles = 0;
    double stand_hold = 4.0, lie_hold = 4.0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-n" || a == "--network") && i + 1 < argc) net = argv[++i];
        else if ((a == "-o" || a == "--out") && i + 1 < argc) out = argv[++i];
        else if (a == "--auto" && i + 1 < argc) auto_cycles = std::stoi(argv[++i]);
        else if (a == "--stand-hold" && i + 1 < argc) stand_hold = std::stod(argv[++i]);
        else if (a == "--lie-hold" && i + 1 < argc) lie_hold = std::stod(argv[++i]);
        else { std::cout << "usage: " << argv[0] << " -n <iface> -o <dir> [--auto N] [--stand-hold s] [--lie-hold s]\n"; return 1; }
    }
    std::signal(SIGINT, on_sigint);
    ChannelFactory::Instance()->Init(0, net);

    // sport mode must be active (this tool never commands motors; it only watches or asks sport mode)
    b2::MotionSwitcherClient msc; msc.SetTimeout(3.0f); msc.Init();
    std::string form, mode;
    if (msc.CheckMode(form, mode) != 0) std::cout << "[warn] motion switcher did not answer (simulator?). Recording anyway.\n";
    else if (mode.empty()) {
        std::cout << "[error] no sport mode is active (the RL controller probably released it).\n"
                     "        Stop it (sudo systemctl stop go2-rl) and reboot the robot so sport mode comes back.\n";
        if (auto_cycles > 0) return 2;
    } else std::cout << "[info] active mode: '" << mode << "'\n";

    std::filesystem::create_directories(out);
    char name[64]; std::time_t now = std::time(nullptr); std::strftime(name, sizeof(name), "sport_motion_%Y-%m-%d_%H-%M-%S.csv", std::localtime(&now));
    const std::string path = out + "/" + name;
    Recorder rec; rec.f = std::fopen(path.c_str(), "w");
    if (!rec.f) { std::cout << "[error] cannot open " << path << "\n"; return 3; }
    rec.header();

    ChannelSubscriber<LowState> sub_low("rt/lowstate");
    sub_low.InitChannel([&](const void* m) { rec.on_lowstate(m); }, 1);
    ChannelSubscriber<SportState> sub_sport("rt/sportmodestate");
    sub_sport.InitChannel([&](const void* m) { rec.on_sport(m); }, 1);

    std::cout << "[info] recording to " << path << " (Ctrl+C to stop)\n";
    sleep_while_running(1.0);
    if (rec.rows == 0) std::cout << "[warn] no rt/lowstate yet. Check the interface name (-n).\n";

    if (auto_cycles > 0) {
        go2::SportClient sport; sport.SetTimeout(10.0f); sport.Init();
        rec.mark("start_lying_hold"); sleep_while_running(2.0);
        for (int c = 1; c <= auto_cycles && g_run; ++c) {
            rec.mark("StandUp_" + std::to_string(c));     int r1 = sport.StandUp();
            if (r1 != 0) std::cout << "[warn] StandUp returned " << r1 << "\n";
            sleep_while_running(stand_hold);
            rec.mark("StandDown_" + std::to_string(c));   int r2 = sport.StandDown();
            if (r2 != 0) std::cout << "[warn] StandDown returned " << r2 << "\n";
            sleep_while_running(lie_hold);
        }
        rec.mark("auto_done"); sleep_while_running(1.0);
        g_run = false;
    } else {
        std::cout << "[info] manual mode: use the remote (sport mode L2+A to stand up / sit down). Ctrl+C when done.\n";
        uint64_t last = 0;
        while (g_run) {
            sleep_while_running(5.0);
            const uint64_t n = rec.rows.load();
            std::printf("[info] %llu rows (%.0f Hz), sport_mode %d, body_height %.3f m\n", (unsigned long long)n,
                        (n - last) / 5.0, rec.sport_mode.load(), rec.body_height.load());
            std::fflush(stdout); last = n;
        }
    }
    { std::lock_guard<std::mutex> lk(rec.mtx); std::fclose(rec.f); rec.f = nullptr; }
    std::cout << "[info] saved " << rec.rows.load() << " rows -> " << path << "\n";
    std::_Exit(0);   // skip DDS teardown races
}
