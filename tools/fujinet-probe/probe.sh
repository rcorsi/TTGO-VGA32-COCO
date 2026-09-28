#!/bin/bash
# Syntax-check (or compile) one fujinet-firmware source file with this project's
# toolchain: Arduino-ESP32 core 2.0.17 = GCC 8.4 + ESP-IDF 4.4 headers.
#
#   tools/fujinet-probe/probe.sh <fujinet-root> <file.cpp> [out.o]
#
# Needs cxx_cmd.txt next to this script: one real g++ compile line captured from
#   arduino-cli compile --fqbn esp32:esp32:esp32wrover:PartitionScheme=huge_app <tiny sketch> -v
# (the line ending in "-o .../p.cpp.o"). See README.md.
D=$(cd "$(dirname "$0")" && pwd); F=$1; SRC=$2; OUT=$3
CMD=$(cat "$D/cxx_cmd.txt"); PRE=${CMD%% -o *}; PRE=${PRE% *}
INC="-I$D/idf44shim -I$F/include $(find "$F/lib" -type d | grep -v '/\.' | sed 's/^/-I/' | tr '\n' ' ')"
DEF="-DDEBUG -DBUILD_COCO -DPINMAP_COCO_DEVKITC -DESP_PLATFORM -DFNIO_IS_STDIO"
STD="-std=gnu++2a -fconcepts -frtti -include $D/idf44shim/fn_compat_gcc8.h"
if [ -n "$OUT" ]; then MODE="-c -o \"$OUT\""; else MODE="-fsyntax-only"; fi
eval "$PRE $STD $MODE -w $DEF $INC \"$SRC\"" 2>&1
