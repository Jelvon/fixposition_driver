#!/bin/bash
# Manual test of CameraStreamer (no ROS, no sensor): builds it with a test harness against a Fixposition SDK build and runs
# it against mock_sensor.py. Not part of the colcon build.
#   ./run_tests.sh <fixposition-sdk-src> <fixposition-sdk-build>      (build dir contains fpsdk_common/libfpsdk_common.so)
# Build the SDK first, e.g.:  cmake <sdk-src> -DFPSDK_USE_FFMPEG=OFF -DFPSDK_USE_PROJ=OFF -DFPSDK_USE_BZ2=OFF && make fpsdk_common
# Needs root (the SDK's CamStream hardcodes port 80), ffmpeg (with libx265) to generate the test stream.
set -eu
SDK=$(readlink -f "$1"); BLD=$(readlink -f "$2"); HERE=$(dirname "$(readlink -f "$0")"); REF=$HERE/../..
W=$(mktemp -d); cd "$W"
FF=0; if grep -q "FPSDK_USE_FFMPEG:BOOL=ON" "$BLD/CMakeCache.txt" 2>/dev/null || grep -q "^FPSDK_USE_FFMPEG:.*=ON" "$BLD/CMakeCache.txt" 2>/dev/null; then FF=1; fi
FLAGS="-std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror -Wshadow -Wunused-parameter -Wformat -Wpointer-arith -Woverloaded-virtual \
  -DFPSDK_USE_FFMPEG=$FF -DFPSDK_USE_ROS1=0 -DFPSDK_USE_ROS2=0 -DFPSDK_USE_BZ2=0 -DFPSDK_USE_PROJ=0 -DNDEBUG \
  -I$REF/include -I$SDK/fpsdk_common/include -I$SDK/fpsdk_common/rosnoros -I/usr/include/eigen3"
g++ $FLAGS -c $REF/src/camera_streamer.cpp -o cs.o
g++ $FLAGS -c $HERE/test_camera_streamer.cpp -o t.o
g++ cs.o t.o -o t -L$BLD/fpsdk_common -lfpsdk_common -Wl,-rpath,$BLD/fpsdk_common -lpthread
ffmpeg -hide_banner -loglevel error -y -f lavfi -i testsrc2=size=1024x768:rate=25 -t 6 -c:v libx265 \
  -x265-params keyint=25:min-keyint=25:bframes=0:repeat-headers=1:log-level=error -f hevc test.h265
DEC=; [ $FF = 1 ] && DEC=decode
run() { local name=$1 secs=$2; shift 2
  python3 -I $HERE/mock_sensor.py --file test.h265 --port 80 "$@" >mock.log 2>&1 & local mp=$!; sleep 0.7
  echo "=== $name"; ./t 127.0.0.1 $secs $DEC 2>&1 | grep -E "RESULT|first frame"; kill $mp; wait $mp 2>/dev/null || true; }
run "A normal"                       4
run "B start mid-GOP (P frames first)" 3 --start-at-p
run "C connection dropped every 30 frames" 6 --drop-after 30
run "D sequence counter jump (no frame lost)" 3 --skip-seq 10
run "E one frame lost"               3 --lose-after 20
run "F stalled connection (open but silent), watchdog 1 s, stop while stalled" 5 --stall-after 50 --stall-secs 60
# Replay of a real recording: fpltool extract -e jsonl some.fpl; then  mock_sensor.py --jsonl some.jsonl --cam CAM1
echo "(FFmpeg decode: $FF)  workdir: $W"
