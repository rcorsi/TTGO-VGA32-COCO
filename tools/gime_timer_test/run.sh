#!/bin/bash
# Build and run the host-side GIME timer test.
# usage: tools/gime_timer_test/run.sh
set -e
D=$(cd "$(dirname "$0")" && pwd)
OUT=${TMPDIR:-/tmp}/gime_timer_test
g++ -std=gnu++11 -O2 -Wall -Wno-unused-function -o "$OUT" "$D/gime_timer_test.cpp"
"$OUT" "$@"
