## RP2040 Driver

A grblHAL driver for the Raspberry Pi Pico RP2040 processor on a [Pi Pico or Pi Pico W board](https://www.raspberrypi.org/products/raspberry-pi-pico/).

This driver can be built with the [Web Builder](https://svn.io-engineering.com:8443/?driver=RP2040&board=PicoCNC).

__Important__ download information can be found [here](https://github.com/grblHAL/core/wiki/Compiling-grblHAL).  
The project has been updated to/now uses [SDK version 2.1.1](https://github.com/raspberrypi/pico-sdk/releases).

The default build environment is Visual Studio Code, it is surprisingly easy to set up on Raspberry Pi - see the [Getting started](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf) documentation for how to for this and other platforms.

> [!NOTE]
> The RP2350B_5X board uses a RP2350B processor, to build the firmware with Visual Studio Code choose the _pimoroni_pga2350_ board.
For other boards choose the _pico_, _pico\_w_, _pico2_ or _pico2\_w_ board that matches the processor or Pico board used.  
The board is selected in the lower right corner of the UI.

---

## This fork (btt_st3215 branch)

Configured for a **BTT SKR Pico 1.0** board (`BOARD_BTT_SKR_PICO_10` in [my_machine.h](my_machine.h)) driving a CNC with a 4th-axis rotary table and a Feetech/Waveshare ST3215 serial bus servo.

> See also [rotary-pico](https://github.com/iyalosovetsky/rotary-pico) — a standalone MicroPython firmware for the same BTT SKR Pico turntable/servo rig, independent of grblHAL.

### Command-line build

[build.sh](build.sh) wraps the CMake/Ninja/Make flow used by the VS Code extension so the firmware can be built from a terminal:

```bash
./build.sh --clean
```

Run `./build.sh --help` for board selection, feature flags (WiFi/Ethernet/Bluetooth/mDNS/MQTT/HPGL), and `PICO_SDK_PATH` overrides. Note: if `~/.pico-sdk/cmake/pico-vscode.cmake` exists (VS Code Pico extension installed), it forces the SDK path to the version pinned in [CMakeLists.txt](CMakeLists.txt) regardless of `--sdk-path`/`PICO_SDK_PATH`.

### 4th axis (A) — rotary table

`N_AXIS` is set to `4` in two places that must be kept in sync:
- [my_machine.h](my_machine.h) (documents the setting, seen by `driver.c` and other project-root sources)
- [CMakeLists.txt](CMakeLists.txt) (`target_compile_definitions(grblHAL PUBLIC N_AXIS=4)`)

Both are required: `grbl/*.c` core sources `#include "config.h"` directly (resolving to `grbl/config.h`, never `my_machine.h`), so `my_machine.h` alone is invisible to them and they'd silently keep the `N_AXIS=3` default — breaking `$376` (rotary axis flag), `$I`'s axis count, and anything else in grbl core that depends on the real axis count.

The rotary table motor connects to the board's 4th motor connector (`M3_STEP_PIN`/`M3_DIRECTION_PIN`/`M3_ENABLE_PIN`, GPIO14/13/15 — see [boards/btt_skr_pico_10_map.h](boards/btt_skr_pico_10_map.h)). After flashing, configure it live:

```
$5=15     ; invert X/Y/Z/A limit pins (NC-switch convention; floating pins read as triggered)
$6=1      ; invert probe
$14=64    ; invert E-stop/control signal
$376=1    ; mark axis A as rotary (units become deg instead of mm)
$103=<steps/degree>   ; calibrate empirically: move a known angle, measure the actual rotation, scale
```

### ST3215 bus servo — M101

[st3215.c](st3215.c) adds `M101 [P<id>] [Q<angle>]` for a Feetech/Waveshare ST3215 servo:
- `M101 Q<angle>` moves the servo (default id `1`, or the id given with `P`) to `<angle>` degrees.
- `M101` (no `Q`) reports status: `[ST3215:<id>|A:<angle>|L:<load%>|V:<voltage>|T:<temperature>]`.

The servo talks over a dedicated hardware UART (UART0, GPIO0 TX / GPIO1 RX by default — free because USB CDC is the primary console stream). ST3215 is a single-wire half-duplex bus: tie TX and RX together (with a series resistor) to the servo's signal pin.

Persistent `$`-settings (NVS-backed, survive reboots and reflashes):
- `$450` — move speed, raw ST3215 steps/s (`0` = max/uncontrolled speed)
- `$451` / `$452` — minimum / maximum allowed angle in degrees; `M101 Q` outside this range is rejected before it reaches the servo

See the header comment in [st3215.c](st3215.c) for the wiring diagram and protocol details.

---
2025-12-31
