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

Configured for a **BTT SKR Pico 1.0** board (`BOARD_BTT_SKR_PICO_10` in [my_machine.h](my_machine.h)) driving a CNC with an independent rotary table and a Feetech/Waveshare ST3215 serial bus servo.

> See also [rotary-pico](https://github.com/iyalosovetsky/rotary-pico) — a standalone MicroPython firmware for the same BTT SKR Pico turntable/servo rig, independent of grblHAL.
>
> [Demo video](https://www.youtube.com/watch?v=oVeflQN5IRw) of the rig in action.

### Command-line build

[build.sh](build.sh) wraps the CMake/Ninja/Make flow used by the VS Code extension so the firmware can be built from a terminal:

```bash
./build.sh --clean
```

Run `./build.sh --help` for board selection, feature flags (WiFi/Ethernet/Bluetooth/mDNS/MQTT/HPGL), and `PICO_SDK_PATH` overrides. Note: if `~/.pico-sdk/cmake/pico-vscode.cmake` exists (VS Code Pico extension installed), it forces the SDK path to the version pinned in [CMakeLists.txt](CMakeLists.txt) regardless of `--sdk-path`/`PICO_SDK_PATH`.

### Rotary table — M102/M103/M104

The table motor connects to the board's 4th motor connector (GPIO14 STEP / GPIO13 DIR / GPIO15 ENABLE — see [boards/btt_skr_pico_10_map.h](boards/btt_skr_pico_10_map.h)), but it is **not** a grbl axis: `N_AXIS` stays `3` (X/Y/Z only).

grbl's G-code motion is a single coordinated multi-axis planner — every block moves all axes together and blocks execute strictly in order. There is no way to have one axis spin indefinitely while X/Y/Z keep accepting and executing independent G-code within that model; queuing the table as a 4th axis meant any "spin forever" move blocked every subsequent X/Y/Z command until it finished. [rotary_table.c](rotary_table.c) drives the table motor directly via a free-running RP2040 PWM slice, entirely outside grbl's stepper segment buffer, so it can turn continuously (or through a timed move) while X/Y/Z G-code keeps running unblocked:

```
M102 [S<pulses/s>] [P<0|1>]   ; start continuous rotation (P0 = CW, P1 = CCW)
M103                          ; stop (also stops it on a soft reset / Ctrl-X)
M104 Q<degrees> [S<pulses/s>] ; rotate by <degrees> (signed, relative), then auto-stop
```

`M104`'s auto-stop is time-based (`duration = steps / rate`, scheduled via `task_add_delayed()`) rather than an exact pulse count, so it doesn't block X/Y/Z either.

**TMC2209 setup.** Being outside `N_AXIS`, the table's TMC2209 (same shared UART bus as X/Y/Z, its own address) is never touched by grbl's own Trinamic driver setup. Left alone it takes its microstep resolution from the MS1/MS2 *pins* (used for UART address selection on this board, not microstepping) instead of the UART-configured MRES register - far coarser than X/Y/Z's 16 microsteps, and audibly rough at low `M102`/`M104` rates. `rotary_table_init()` explicitly adds it as a TMC2209 motor (`TMC2209_AddMotor`) and configures it the same way grbl's settings load does for X/Y/Z. Current and chopper algorithm are runtime `$`-settings, applied immediately on change, no reboot needed:

- `$453` - motor current, mA RMS (0-1500, capped to the fitted 42BYGH34's rating - check your own motor's rating before raising it)
- `$454` - chopper mode: `0` = StealthChop (quiet, default), `1` = SpreadCycle (louder, more torque)

> [!WARNING]
> Don't call the settings `.load()` handler (or anything that writes to the TMC2209 over UART) directly from `rotary_table_init()`/`board_init()`. It hung the board (reproduced by testing) - most likely because going through `restore()` can trigger an NVS flash write (`flash_range_erase`/`flash_range_program`) this early in boot, before whatever needs to be quiesced for that to be safe on RP2040 actually is. grbl's settings framework calls each plugin's `.load` on its own at a safe point later in boot regardless - same as [st3215.c](st3215.c), which never calls its own load function directly either - so it isn't even necessary to force it early. `TMC2209_AddMotor` already leaves the driver in a safe working state (default current, StealthChop) for the window before that happens.

Default direction invert (`$3`) and X/Y/Z limit/probe/E-stop invert are still configured live after flashing:

```
$5=7      ; invert X/Y/Z limit pins (NC-switch convention; floating pins read as triggered)
$6=1      ; invert probe
$14=64    ; invert E-stop/control signal
```

`$3` (direction invert) defaults to `7` (X/Y/Z inverted) out of the box via `DEFAULT_DIR_SIGNALS_INVERT_MASK` in [CMakeLists.txt](CMakeLists.txt) — not `my_machine.h`, because `grbl/settings.c` (which actually reads it) `#include`s `grbl/config.h` directly and never sees `my_machine.h`. The same reach problem applies to any other compile-time default that grbl *core* (not just driver.c) needs to see — put those in `CMakeLists.txt`, not `my_machine.h`.

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
