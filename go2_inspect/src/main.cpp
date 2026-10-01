// READ-ONLY: no publishers
//
// go2_inspect — Unitree Go2 상태 확인 도구 (읽기 전용)
//
//   * rt/lowstate, rt/wirelesscontroller 를 ChannelSubscriber 로 구독만 한다.
//   * 발행 채널, RPC 를 사용하는 객체를 전혀 만들지 않는다.
//   * 로봇에 어떤 명령도 보내지 않는다.
//
// 사용법: go2_inspect <domain_id> <network_interface> [--mode monitor|guided] [--output FILE]
#include "app.hpp"
#include "state_cache.hpp"
#include "terminal.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

void PrintUsage(std::ostream& os, const char* prog) {
  os << "사용법: " << prog
     << " <domain_id> <network_interface> [--mode monitor|guided] [--output FILE]\n"
        "\n"
        "  READ-ONLY: rt/lowstate, rt/wirelesscontroller 를 구독만 합니다.\n"
        "             어떤 토픽에도 발행하지 않으며 로봇에 명령을 보내지 않습니다.\n"
        "\n"
        "  domain_id          DDS 도메인 ID (Go2 기본값 0)\n"
        "  network_interface  로봇 내부망(192.168.123.0/24)에 연결된 네트워크 인터페이스 이름\n"
        "                     `ip a` 로 확인하세요: inet 192.168.123.x 가 붙은 인터페이스 (예: eth0)\n"
        "  --mode monitor     실시간 모니터 (기본값). 키: r = 기준값 재설정, q = 종료\n"
        "  --mode guided      안내형 확인 모드. 결과를 YAML 로 저장\n"
        "  --output FILE      안내형 모드 결과 파일 (기본값: go2_calibration.yaml)\n"
        "  -h, --help         이 도움말\n"
        "\n"
        "예시:\n"
        "  " << prog << " 0 eth0\n"
        "  " << prog << " 0 eth0 --mode guided\n"
        "\n"
        "주의: 측정 전 로봇을 바닥에 엎드린 댐핑 상태로 두세요.\n";
}

void PrintArgError(const char* prog, const std::string& msg) {
  std::cerr << "오류: " << msg << "\n\n";
  PrintUsage(std::cerr, prog);
  std::cerr << "\n힌트: 네트워크 인터페이스 이름은 `ip a` 로 확인하세요.\n";
}

}  // namespace

int main(int argc, char** argv) {
  const char* prog = argv[0];

  // --help 는 DDS 초기화 전에 처리 (로봇 없이도 동작)
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
      PrintUsage(std::cout, prog);
      return 0;
    }
  }

  AppConfig cfg;
  std::string mode = "monitor";
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--mode" || a == "--output" || a == "-o") {
      if (i + 1 >= argc) {
        PrintArgError(prog, a + " 뒤에 값이 필요합니다.");
        return 1;
      }
      (a == "--mode" ? mode : cfg.output_path) = argv[++i];
    } else if (a.rfind("--mode=", 0) == 0) {
      mode = a.substr(7);
    } else if (a.rfind("--output=", 0) == 0) {
      cfg.output_path = a.substr(9);
    } else if (!a.empty() && a[0] == '-' && a.size() > 1 && !std::isdigit(static_cast<unsigned char>(a[1]))) {
      PrintArgError(prog, "알 수 없는 옵션: " + a);
      return 1;
    } else {
      positional.push_back(a);
    }
  }

  if (positional.size() < 2) {
    PrintArgError(prog, "domain_id 와 network_interface 가 필요합니다.");
    return 1;
  }
  if (positional.size() > 2) {
    PrintArgError(prog, "인자가 너무 많습니다: " + positional[2]);
    return 1;
  }
  {
    char* end = nullptr;
    const long v = std::strtol(positional[0].c_str(), &end, 10);
    if (end == positional[0].c_str() || *end != '\0' || v < 0 || v > 232) {
      PrintArgError(prog, "domain_id 는 0~232 사이 정수여야 합니다: " + positional[0]);
      return 1;
    }
    cfg.domain_id = static_cast<int>(v);
  }
  cfg.iface = positional[1];
  if (mode != "monitor" && mode != "guided") {
    PrintArgError(prog, "--mode 는 monitor 또는 guided 여야 합니다: " + mode);
    return 1;
  }

  term::InstallSignalHandlers();

  std::cout << "go2_inspect  READ-ONLY (no publishers)  domain=" << cfg.domain_id
            << " iface=" << cfg.iface << " mode=" << mode << std::endl;

  StateCache cache;
  try {
    unitree::robot::ChannelFactory::Instance()->Init(cfg.domain_id, cfg.iface);
    cfg.start_time = Clock::now();
    cache.Start();
  } catch (const std::exception& e) {
    std::cerr << "DDS 초기화 실패: " << e.what() << "\n"
              << "인터페이스 이름(`ip a`), 도메인 ID, 랜선, IP(192.168.123.x) 를 확인하세요.\n";
    return 1;
  }

  int rc = 0;
  if (mode == "monitor") {
    rc = RunMonitor(cfg, cache);
  } else {
    rc = RunGuided(cfg, cache);
  }

  cache.Stop();
  return rc;
}
