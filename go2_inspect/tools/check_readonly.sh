#!/usr/bin/env bash
# READ-ONLY: no publishers
# go2_inspect 에 발행자/명령 경로가 없는지 확인한다.
#   1) 소스 grep  2) (빌드 후) 오브젝트 파일 심볼 검사
# 사용: ./tools/check_readonly.sh [build_dir]
set -u
cd "$(dirname "$0")/.."
BUILD_DIR="${1:-build}"
fail=0

echo "== [1] 모든 소스 상단에 'READ-ONLY: no publishers' 표기 =="
for f in src/*; do
  if head -n 3 "$f" | grep -q "READ-ONLY: no publishers"; then
    echo "  OK      $f"
  else
    echo "  MISSING $f"; fail=1
  fi
done

echo
echo "== [2] 금지 항목 grep (결과가 비어 있어야 함) =="
PATTERN='ChannelPublisher|channel_publisher|CreateSendChannel|Publish|->Write\(|\.Write\(|LowCmd|lowcmd|SportClient|MotionSwitcher|RobotStateClient|VuiClient|ObstaclesAvoidClient|Client\b|_client\.hpp'
if grep -rnE "$PATTERN" src/; then
  echo "  !!! 금지 항목 발견"; fail=1
else
  echo "  (없음)"
fi

echo
echo "== [3] 사용 중인 채널 객체 (구독자만 있어야 함) =="
grep -rnE "Channel(Subscriber|Publisher)<" src/ || true

if [ -d "$BUILD_DIR" ]; then
  echo
  echo "== [4] 오브젝트 파일 심볼 검사: $BUILD_DIR (결과가 비어 있어야 함) =="
  objs=$(find "$BUILD_DIR" -path '*go2_inspect.dir*' -name '*.o')
  if [ -z "$objs" ]; then
    echo "  오브젝트 파일 없음 (먼저 빌드하세요)"
  else
    hits=$(nm -C $objs 2>/dev/null | grep -E "ChannelPublisher|DdsWriter|LowCmd_|SportClient|MotionSwitcherClient" || true)
    if [ -n "$hits" ]; then
      echo "$hits"; echo "  !!! 발행 관련 심볼 발견"; fail=1
    else
      echo "  (없음)"
    fi
  fi
fi

echo
if [ $fail -eq 0 ]; then echo "결과: READ-ONLY 확인 통과"; else echo "결과: 확인 실패"; fi
exit $fail
