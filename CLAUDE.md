# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository overview

This is a hardware/firmware project (not a typical software repo). It contains schematics, PCB board files, gerbers, and Arduino firmware for every hardware revision of the "Select-Fire Rapidstrike Kit" — an open-source control board that adds select-fire (safety/semi-auto/burst/full-auto) capability to a Nerf Rapidstrike blaster, with optional rate-of-fire control. Sold at https://suild.com/shop/4.

There is no build system, package manager, or test runner in this repo — it's a collection of EAGLE CAD hardware files and Arduino `.ino` sketches. There is nothing to `npm install`, compile via CI, or lint. "Testing" the firmware means compiling/uploading it to real hardware via the Arduino IDE or PlatformIO.

## Repository layout

Each top-level directory is a distinct hardware revision, oldest to newest: `v0.1`, `v0.2`, `v1.0`, `v1.1`, `v1.2`, `v1.3`. Revisions are independent snapshots, not branches of shared code — files are duplicated and diverge across versions rather than refactored in place. When asked to change firmware behavior, always confirm *which version's* directory the user means; do not assume changes should propagate to other versions.

Within a version directory:
- `Hardware/` (or `hardware/` — casing is inconsistent across versions, e.g. `v1.2/hardware` vs `v1.3/Hardware`): EAGLE `.sch` (schematic) and `.brd` (board) files, a rendered `Schematics.pdf`, top/bottom PCB renders as PNGs, and `gerbers.zip` (fab-ready Gerber output). `.epf` is the EAGLE project file (present from v1.1 onward).
- `Software/select-fire-rapidstrike-firmware/`: the production `.ino` sketch for that hardware revision. Only `v0.1` and `v1.0` have firmware; later hardware revisions (`v1.1`–`v1.3`) did not get accompanying firmware changes in this repo.
- `v1.0/Software/select-fire-rapidstrike-firmware-rp2040/`: a port of the v1.0 firmware to a Waveshare RP2040-Zero (different MCU, not a new PCB revision). Same control logic, retargeted pins and board-specific constants — see the README in that directory for the full pin map and hardware caveats (3.3V logic, gate-drive voltage, current-sense scaling) before wiring an RP2040-Zero into a v1.0 board.
- `Software/tests/`: standalone throwaway `.ino` sketches used to bring up/validate individual subsystems in isolation (rotary switch reading, half-bridge driving, serial + digital I/O, a solenoid-only variant of the main firmware, etc.) — not automated tests, and not exercised by any test runner.

`Wiring Diagrams/` (top-level, version-independent) holds reference wiring diagrams (JPG) for common builds, e.g. flywheel motors with a microswitch, and a voltmeter with a kill switch.

## Firmware architecture (v1.0, the current production sketch)

`v1.0/Software/select-fire-rapidstrike-firmware/select-fire-rapidstrike-firmware.ino` is the most complete/representative firmware and a good reference for the general design (v0.1's version is an earlier, more verbosely-commented draft of the same logic using the older `Button` library instead of `JC_Button`).

Key design points, since the whole sketch is one flat file and the control flow spans many functions:

- **State lives in three global structs**, not scattered variables: `firingState` (fire mode, trigger/cycle-switch state, darts fired/to-fire, rate of fire, pusher/solenoid on/off timing), `fetState` (current vs. target high/low-side MOSFET state, shoot-through protection), `currentSenseState` (current-sense sampling buffers, overcurrent-protection counters).
- **Pusher drive is a target/current state machine**, not direct pin writes: code sets `firingState.targetPusherState` (`OFF` / `BRAKE` / `ON`); `controlMotors()` (called every `loop()`) is the only place that actually transitions the FETs, and it does so through `handleComplementaryFETTransition()` with a `SHOOT_THROUGH_DELAY` timer to avoid both high- and low-side FETs conducting simultaneously (shoot-through). Never set FET pins directly from firing logic — always go through `targetPusherState`.
- **Two mutually-exclusive firing mechanisms** share the same state machine: pusher-motor firing (`handlePusherMotorFiring`) and solenoid firing (`handleSolenoidFiring`), selected at runtime by reading a jumper (`PUSHER_MECHANISM_SELECT_PIN`, active-low = solenoid). Solenoid timing is duty-cycle based (`SOLENOID_DUTY_CYCLE`, `solenoidOnTime`/`solenoidOffTime`); pusher motor timing is a simple off-time derived from rate of fire.
- **Fire mode** (`SAFETY` / `SEMI_AUTO` / `BURST_FIRE` / `FULL_AUTO`) is read from a 4-position rotary switch (one pin per position, active-low) at the moment the trigger is pulled, not continuously polled.
- **Rate of fire** comes from a potentiometer (`POT_PIN`), remapped to a different range depending on whether the pusher motor or solenoid mechanism is active.
- **Overcurrent protection** is a separate concern layered on top: `monitorCurrent()` runs on its own timer, computes current via `analogReadingToVoltage()`/`voltageToCurrent()` (based on `SENSE_RESISTANCE`), keeps a rolling differentiation buffer (`CircularBuffer`) to compute dI/dt, and trips `currentSenseState.isTooMuchCurrent` after `MAX_OCP_SAMPLES` consecutive over-limit readings — checked first thing in `controlMotors()`, which unconditionally cuts power when tripped.
- Everything is non-blocking: all delayed actions (shoot-through delay, solenoid on/off timing, pusher motor off-time, current sampling) go through the `arduino-timer` library (`Timer<>` instances, `.tick()`d every `loop()`), not `delay()`.

Dependencies (Arduino libraries, installed via Arduino IDE Library Manager or PlatformIO, not vendored in this repo): `JC_Button`, `CircularBuffer`, `arduino-timer` (`timer.h`). v0.1 additionally uses the older `Button` library and `SoftwareSerial` in some test sketches.

The firmware source comments note a preference for PlatformIO over the Arduino IDE for development, though no PlatformIO project files (`platformio.ini`) currently exist in this repo — sketches are structured as plain `.ino` files compilable by either toolchain.
