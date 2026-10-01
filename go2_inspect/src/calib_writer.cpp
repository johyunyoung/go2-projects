// READ-ONLY: no publishers
//
// 측정 결과를 yaml-cpp 로 go2_calibration.yaml 에 저장한다.
// 가설과 다른 항목에는 "# MISMATCH" 주석을 붙인다.
#include "calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace {

void EmitVec3(YAML::Emitter& out, const Vec3& v) {
  out << YAML::Flow << YAML::BeginSeq << v.x << v.y << v.z << YAML::EndSeq;
}

template <size_t N>
void EmitArr(YAML::Emitter& out, const std::array<double, N>& a) {
  out << YAML::Flow << YAML::BeginSeq;
  for (double v : a) out << v;
  out << YAML::EndSeq;
}

void EmitStrings(YAML::Emitter& out, const std::vector<std::string>& v) {
  out << YAML::BeginSeq;
  for (const auto& s : v) out << s;
  out << YAML::EndSeq;
}

void EmitJoints(YAML::Emitter& out, const CalibrationData& d) {
  out << YAML::Key << "joints" << YAML::Value << YAML::BeginMap;
  for (const auto& j : d.joints) {
    out << YAML::Key << j.name << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "index" << YAML::Value;
    if (j.measured) {
      out << j.index;
      if (j.Mismatch()) {
        out << YAML::Comment("!!! MISMATCH vs hypothesis: 가설 index " + std::to_string(j.hyp_index) +
                             " (" + j.name + "), 측정 index " + std::to_string(j.index) +
                             " (가설상 " + hyp::kJointName[j.index] + ")");
      }
    } else {
      out << YAML::Null << YAML::Comment("미측정");
    }
    out << YAML::Key << "sign" << YAML::Value;
    if (j.measured) out << j.sign; else out << YAML::Null;
    out << YAML::Key << "measured_delta_rad" << YAML::Value;
    if (j.measured) out << j.peak_delta; else out << YAML::Null;
    out << YAML::Key << "final_delta_rad" << YAML::Value;
    if (j.measured) out << j.final_delta; else out << YAML::Null;
    out << YAML::Key << "hypothesis_index" << YAML::Value << j.hyp_index;
    if (j.measured && j.second_index >= 0) {
      out << YAML::Key << "second_largest" << YAML::Value << YAML::Flow << YAML::BeginMap
          << YAML::Key << "index" << YAML::Value << j.second_index
          << YAML::Key << "delta_rad" << YAML::Value << j.second_delta << YAML::EndMap;
    }
    if (!j.warnings.empty()) {
      out << YAML::Key << "warnings" << YAML::Value;
      EmitStrings(out, j.warnings);
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
}

void EmitImu(YAML::Emitter& out, const CalibrationData& d) {
  const ImuResult& r = d.imu;
  out << YAML::Key << "imu" << YAML::Value << YAML::BeginMap;

  out << YAML::Key << "quaternion_order" << YAML::Value;
  if (r.measured && !r.order.empty()) {
    out << r.order;
    if (r.Mismatch()) {
      out << YAML::Comment(std::string("!!! MISMATCH vs hypothesis: 가설 ") + hyp::kQuaternionOrder +
                           ", 측정 " + r.order);
    }
  } else {
    out << YAML::Null << YAML::Comment("판정 불가 / 미측정");
  }
  out << YAML::Key << "decision" << YAML::Value
      << (!r.measured ? "not_measured" : (r.confident ? "confident" : "uncertain"));
  out << YAML::Key << "hypothesis_order" << YAML::Value << hyp::kQuaternionOrder;

  out << YAML::Key << "flat_angle_to_down_deg" << YAML::Value << YAML::Flow << YAML::BeginMap
      << YAML::Key << "wxyz" << YAML::Value << r.flat_err_deg[0]
      << YAML::Key << "xyzw" << YAML::Value << r.flat_err_deg[1] << YAML::EndMap;
  out << YAML::Key << "acc_agreement_deg" << YAML::Value << YAML::Flow << YAML::BeginMap
      << YAML::Key << "wxyz" << YAML::Value << r.acc_err_deg[0]
      << YAML::Key << "xyzw" << YAML::Value << r.acc_err_deg[1] << YAML::EndMap;
  out << YAML::Key << "tilt_sign_ok" << YAML::Value << YAML::Flow << YAML::BeginMap
      << YAML::Key << "wxyz" << YAML::Value << r.tilt_sign_ok[0]
      << YAML::Key << "xyzw" << YAML::Value << r.tilt_sign_ok[1] << YAML::EndMap;
  if (!r.notes.empty()) {
    out << YAML::Key << "notes" << YAML::Value;
    EmitStrings(out, r.notes);
  }

  out << YAML::Key << "poses" << YAML::Value << YAML::BeginMap;
  for (const auto& p : r.poses) {
    out << YAML::Key << p.key << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "description" << YAML::Value << p.description;
    out << YAML::Key << "measured" << YAML::Value << p.measured;
    if (p.measured) {
      out << YAML::Key << "samples" << YAML::Value << p.samples;
      out << YAML::Key << "quaternion_raw" << YAML::Value;
      EmitArr(out, p.quat);
      out << YAML::Key << "rpy_rad" << YAML::Value;
      EmitArr(out, p.rpy);
      out << YAML::Key << "gyroscope" << YAML::Value;
      EmitArr(out, p.gyro);
      out << YAML::Key << "accelerometer" << YAML::Value;
      EmitArr(out, p.acc);
      out << YAML::Key << "gravity_body_wxyz" << YAML::Value;
      EmitVec3(out, p.g_wxyz);
      out << YAML::Key << "gravity_body_xyzw" << YAML::Value;
      EmitVec3(out, p.g_xyzw);
      out << YAML::Key << "gravity_from_acc" << YAML::Value;
      EmitVec3(out, p.g_acc);
    }
    if (!p.warnings.empty()) {
      out << YAML::Key << "warnings" << YAML::Value;
      EmitStrings(out, p.warnings);
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;  // poses

  out << YAML::EndMap;  // imu
}

void EmitButtons(YAML::Emitter& out, const CalibrationData& d) {
  // 요구 형식: 버튼 이름 → 비트 번호
  out << YAML::Key << "remote_keys" << YAML::Value << YAML::BeginMap;
  for (const auto& b : d.buttons) {
    out << YAML::Key << b.name << YAML::Value;
    if (b.measured) {
      out << b.bit;
      if (b.Mismatch()) {
        out << YAML::Comment("!!! MISMATCH vs reference hypothesis: 가설 bit " +
                             std::to_string(b.hyp_bit) + ", 측정 bit " + std::to_string(b.bit));
      }
    } else {
      out << YAML::Null << YAML::Comment("미측정");
    }
  }
  out << YAML::EndMap;

  // 상세 정보
  out << YAML::Key << "remote_keys_detail" << YAML::Value << YAML::BeginMap;
  for (const auto& b : d.buttons) {
    out << YAML::Key << b.name << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "hypothesis_bit" << YAML::Value << b.hyp_bit;
    out << YAML::Key << "source" << YAML::Value << b.source;
    out << YAML::Key << "other_bits" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int v : b.other_bits) out << v;
    out << YAML::EndSeq;
    if (!b.warnings.empty()) {
      out << YAML::Key << "warnings" << YAML::Value;
      EmitStrings(out, b.warnings);
    }
    out << YAML::EndMap;
  }
  out << YAML::EndMap;
}

}  // namespace

bool SaveCalibrationYaml(const std::string& path, const CalibrationData& d, std::string* err) {
  YAML::Emitter out;
  out.SetDoublePrecision(6);
  out.SetFloatPrecision(6);

  const int mismatches = d.MismatchCount();
  out << YAML::Comment("go2_calibration.yaml - go2_inspect 측정 결과 (READ-ONLY 도구, 로봇에 명령 미발행)");
  out << YAML::Newline;
  out << YAML::Comment("joints.<name>.sign: 지시 방향으로 움직였을 때 q 변화 부호 "
                       "(hip=몸 바깥쪽 벌림, thigh=다리를 앞으로 들어 올림, calf=무릎 더 접기)");
  out << YAML::Newline;
  out << YAML::Comment("joints.<name>.measured_delta_rad: 측정 중 기준값 대비 최대 변화량 (부호 포함)");
  out << YAML::Newline;
  out << YAML::Comment("remote_keys: 비교용 가설은 unitree_sdk2 example/wireless_controller 의 비트필드 순서");
  out << YAML::Newline;
  if (mismatches > 0) {
    out << YAML::Comment("!!!!! 가설과 다른 항목 " + std::to_string(mismatches) +
                         "개 — 'MISMATCH' 주석을 확인하세요 !!!!!");
    out << YAML::Newline;
  }
  if (!d.complete) {
    out << YAML::Comment("!!!!! 측정이 중간에 중단된 부분 결과입니다 !!!!!");
    out << YAML::Newline;
  }

  out << YAML::BeginMap;

  out << YAML::Key << "meta" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "measured_at" << YAML::Value << d.measured_at;
  out << YAML::Key << "network_interface" << YAML::Value << d.iface;
  out << YAML::Key << "domain_id" << YAML::Value << d.domain_id;
  out << YAML::Key << "tool" << YAML::Value << "go2_inspect (read-only, no publishers)";
  out << YAML::Key << "complete" << YAML::Value << d.complete;
  out << YAML::Key << "steps_done" << YAML::Value << YAML::Flow << YAML::BeginMap
      << YAML::Key << "joints" << YAML::Value << d.joints_done
      << YAML::Key << "imu" << YAML::Value << d.imu_done
      << YAML::Key << "buttons" << YAML::Value << d.buttons_done << YAML::EndMap;
  out << YAML::Key << "hypothesis_mismatch_count" << YAML::Value << mismatches;
  if (mismatches > 0) out << YAML::Comment("!!! 가설과 다른 항목 있음");
  out << YAML::EndMap;

  EmitJoints(out, d);
  EmitImu(out, d);
  EmitButtons(out, d);

  if (!d.warnings.empty()) {
    out << YAML::Key << "warnings" << YAML::Value;
    EmitStrings(out, d.warnings);
  }

  out << YAML::EndMap;

  if (!out.good()) {
    if (err) *err = "yaml emitter error: " + out.GetLastError();
    return false;
  }

  std::ofstream f(path);
  if (!f) {
    if (err) *err = "파일을 열 수 없습니다: " + path;
    return false;
  }
  f << out.c_str() << "\n";
  if (!f) {
    if (err) *err = "파일 쓰기 실패: " + path;
    return false;
  }
  return true;
}
