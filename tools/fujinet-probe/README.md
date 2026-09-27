# fujinet-probe — can fujinet-firmware build with our toolchain?

Phase 0 of `coco3/fuji-net-plan.md`. Upstream FujiNet builds with PlatformIO
espressif32 6.x (ESP-IDF 5, GCC 12, C++20); this project is locked to Arduino-ESP32
core 2.0.17 (ESP-IDF 4.4, GCC 8.4) by FabGL. `probe.sh` compiles FujiNet sources
with our exact compiler line plus:

- `-std=gnu++2a -fconcepts -frtti` — GCC 8's Concepts-TS mode accepts FujiNet's
  C++20 `concept`/`requires`; `-frtti` for its few `dynamic_cast`s.
- `idf44shim/fn_compat_gcc8.h` (force-included): `std::derived_from`,
  `UART_SCLK_DEFAULT`.
- `idf44shim/soc/gpio_num.h`: IDF 5.1 header mapped to IDF 4.4's `hal/gpio_types.h`.

## Setup

Capture one real compile line (it is machine-specific, so it is git-ignored):

```bash
mkdir -p /tmp/probe && printf 'void setup(){}\nvoid loop(){}\n' > /tmp/probe/probe.ino
printf '#include <Arduino.h>\nint probe_x;\n' > /tmp/probe/p.cpp
arduino-cli compile --fqbn esp32:esp32:esp32wrover:PartitionScheme=huge_app /tmp/probe -v 2>&1 \
  | grep -m1 -- '-o [^ ]*p\.cpp\.o' > tools/fujinet-probe/cxx_cmd.txt
```

## Use

```bash
tools/fujinet-probe/probe.sh ~/proyectos/fujinet-firmware lib/bus/drivewire/drivewire.cpp        # syntax only
tools/fujinet-probe/probe.sh ~/proyectos/fujinet-firmware lib/bus/drivewire/drivewire.cpp x.o    # compile
```

## Result (2026-09-27, fujinet-firmware `c4676f1`)

52 of 62 files in the Phase 4–6 subset (`lib/bus/drivewire`, `lib/device/drivewire`,
`lib/fuji`, `lib/network-protocol`, `lib/TNFSlib`, `lib/fnjson`, `lib/utils`,
`lib/compat`) pass after one source patch (`ESP32UARTChannel.h`: drop the IDF 5
`uart_config_t.flags` initializer). The other 10: SSH/SMB/NFS (need absent libs; out
of scope), `fnWebSocketClient` (IDF 5 `crt_bundle_attach`), `NetworkProtocolFactory`
and `fujiHost` (pull in the SMB/SSH headers), `drivewire.cpp:407` (`std::map::contains`).
The 33 in-scope objects: ~152 KB code, 6.4 KB `.bss` (5 KB is the global
`platformFuji` object — heap-allocate it so it lands in PSRAM).
Not yet probed: `lib/http`, `lib/FileSystem`, `lib/config`, `lib/hardware` (to be
replaced by the shim), and linking.
