#!/usr/bin/env bash
# this arg has moved to settings.json
#-c "--board Bela --analog-channels 0 --digital-channels 16 --verbose --mute-speaker yes --use-analog no --use-digital yes --disable-led --stop-button-pin -1 --period 16 --high-performance-mode" \
# BBB_HOSTNAME="192.168.1.2" ../Bela/scripts/build_project.sh \
BBB_ADDRESS="root@192.168.42.101" ../Bela/scripts/build_project.sh \
  -p bela-lsl-timing \
  -c "--analog-channels 2 --digital-channels 16 --audio-input-gain 0 --line-out-level 0 --hp-level 0 --verbose --mute-speaker yes --use-analog no --use-digital yes --disable-led --period 16 --high-performance-mode" \
  --force \
  -m "CPPFLAGS='-std=c++1z -I/root/Bela/projects/bela-lsl-timing/include' LDLIBS=/root/Bela/projects/bela-lsl-timing/lib/liblsl.so LDFLAGS='-Wl,-rpath,/root/Bela/projects/bela-lsl-timing/lib'" \
  ./
