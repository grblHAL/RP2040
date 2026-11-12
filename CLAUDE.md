# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A [grblHAL](https://github.com/grblHAL) driver for the Raspberry Pi Pico (RP2040), forked and configured for a specific machine on a **BTT SKR Pico 1.0** board: a 3-axis CNC (X/Y/Z, `BOARD_BTT_SKR_PICO_10` in [my_machine.h](my_machine.h)) plus two custom additions bolted onto the driver: a Feetech/Waveshare **ST3215 serial bus servo** and an independently-driven **rotary table** stepper. See [README.md](README.md) for the full write-up of both, including wiring notes.

Everything under `grbl/`, `motors/`, `spindle/`, `sdcard/`, `keypad/`, `bluetooth/`, `networking/`, `webui/`, `embroidery/`, `fans/`, `laser/`, `plugins/`, `plasma/`, `eeprom/`, `trinamic/` is a **git submodule** pulled from the upstream grblHAL org — treat it as vendored code, not something to casually edit. Everything else at the repo root (`driver.c`, `boards/`, `st3215.c`, `rotary_table.c`, `my_machine.h`, `CMakeLists.txt`, ...) is this driver/fork's own code.

## Build

```bash
./build.sh --clean          # full clean build -> build/grblHAL.uf2
./build.sh                  # incremental
./build.sh --help           # board selection, feature flags, PICO_SDK_PATH override
```

`build.sh` wraps the same CMake configure/build flow the VS Code Pico extension uses (Ninja if present, else Make), so it can be driven from a terminal instead. If `~/.pico-sdk/cmake/pico-vscode.cmake` exists (VS Code Pico extension installed), it silently forces the SDK path to the version pinned in [CMakeLists.txt](CMakeLists.txt) regardless of `--sdk-path`/`$PICO_SDK_PATH`.

Submodules must be checked out at the commits the superproject expects (not just "present") or the build fails with confusing API-mismatch errors (e.g. missing struct fields) because `driver.c` et al. are written against a newer/older `grbl` core than what's on disk:

```bash
git submodule update --init --recursive
```

There is no lint/test suite — this is embedded firmware; correctness is verified by flashing and exercising the machine.

### Flashing

No `picotool`-based one-shot flash: this build's USB descriptors don't expose the reset-to-BOOTSEL interface, so `picotool load -f` fails. Flash manually:

1. Hold BOOTSEL, plug in / reset the board, release BOOTSEL — it enumerates as a `RPI-RP2` USB mass-storage device.
2. Copy `build/grblHAL.uf2` onto it. The board erases/reboots into the new firmware automatically once the write completes.

Occasionally a copy doesn't fully land (board comes back up in BOOTSEL again, or the whole board seems to vanish from USB for a while) — retry the BOOTSEL cycle rather than assuming the firmware itself is broken.

## Architecture gotcha: `my_machine.h` doesn't reach grbl core

This is the single most important thing to know about this codebase, and it's cost real debugging time twice already (see git log for `N_AXIS` and `DEFAULT_DIR_SIGNALS_INVERT_MASK`).

- Project-root files (`driver.c`, `st3215.c`, `rotary_table.c`, ...) `#include "driver.h"`, which pulls in `my_machine.h` early. They see everything defined there.
- `grbl/*.c` core files `#include "config.h"` (unqualified), which the compiler resolves to `grbl/config.h` — a file in the *same directory* as the including file — before ever considering the project root's include path. **They never see `my_machine.h`.**

So a `#define` in `my_machine.h` only configures the driver-side files; anything that grbl *core* itself needs to see (axis count, default settings values baked into `grbl/settings.c`'s init struct, etc.) must instead be injected as a compiler flag in `CMakeLists.txt`:

```cmake
target_compile_definitions(grblHAL PUBLIC DEFAULT_DIR_SIGNALS_INVERT_MASK=7)
```

`-D` flags apply uniformly to every translation unit regardless of its own include chain, which is why this works and a header `#define` doesn't. If you add a new compile-time option and grbl core's behavior doesn't reflect it (default settings wrong after a fresh boot, a `$`-setting reporting `Status_SettingDisabled` when it shouldn't, etc.), check whether it's a grbl-core-visible symbol that belongs in `CMakeLists.txt` instead.

## Architecture: custom plugins live outside grbl's axis system

The rotary table is deliberately **not** a grbl axis (`N_AXIS` is 3: X/Y/Z only), even though it started out as one. grbl's G-code motion is a single coordinated multi-axis planner — every block moves all configured axes together and blocks execute strictly in order — so there is no way to have one axis spin indefinitely while other axes keep accepting and executing independent G-code within that model. [rotary_table.c](rotary_table.c) drives the table's stepper directly via a free-running RP2040 PWM slice (M102/M103/M104), entirely outside grbl's stepper segment buffer, so X/Y/Z G-code is never blocked by it. The servo ([st3215.c](st3215.c), M101) is UART-driven and was never a grbl axis at all.

Consequences of a motor living outside `N_AXIS`:
- It's invisible to grbl's own Trinamic driver setup, which only configures the first `N_AXIS` TMC2209s. A motor added this way must be configured explicitly via `TMC2209_AddMotor()` (see `rotary_table_init()`) or it's left on power-on-reset defaults (wrong microstepping, wrong current) — this caused audibly rough/jerky motion until fixed.
- Never call anything that writes to NVS flash (`settings_restore()`/`settings_save()`, or a plugin's own `.load()` handler with a fresh/corrupt NVS region) directly from `board_init()`. Doing so hung the board (reproduced by testing) — most likely `flash_range_erase()`/`flash_range_program()` isn't safe to call this early in boot. grbl's settings framework calls each registered plugin's `.load()` on its own at a safe point later; let it.

## Custom M-codes (this fork's own plugins)

Both use `UserMCode_Generic1..4` (M101-M104), explicitly reserved in `grbl/gcode.h` for private/non-public use, and both reserve `Setting_UserDefined_0..9` (`$450`-`$459`, also explicitly reserved for private plugins) for their own persistent settings — see `grbl/settings.h` before picking new IDs.

- **M101** ([st3215.c](st3215.c)) — `M101 [P<id>] [Q<angle>]`: move/query the ST3215 servo. `$450`-`$452` control speed and angle clamping.
- **M102/M103/M104** ([rotary_table.c](rotary_table.c)) — start/stop/timed-move for the rotary table. `$453`/`$454` control motor current and TMC2209 chopper mode (StealthChop/SpreadCycle), applied live via the `tmchal_t` handle, no reboot needed.

Both append fields to the `?` realtime status report via `grbl.on_realtime_report` chaining (`|ST3215:<angle>`, `|TBL:<angle>|TBLABS:<absolute angle>`). The servo's field is backed by a periodic background poll (`task_add_delayed`, ~500ms), not a live UART read on every `?`, so it doesn't add round-trip latency to status polling.

## Board-specific pin map

Only one board is active at a time via `my_machine.h`'s `BOARD_*` defines; the corresponding `boards/<board>_map.h` supplies pin assignments and `boards/<board>.c` supplies `board_init()` (called from `driver.c`, guarded by `#ifdef HAS_BOARD_INIT`). This is the hook point both custom plugins' `_init()` functions are called from — follow that pattern for anything else that needs one-time setup tied to this specific board.
