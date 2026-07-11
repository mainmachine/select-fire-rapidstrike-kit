# RP2040-Zero port

A port of the [v1.0 firmware](../select-fire-rapidstrike-firmware) from the Arduino Nano to a [Waveshare RP2040-Zero](https://www.waveshare.com/wiki/RP2040-Zero). Control logic is unchanged — this is a pin/board retarget, not a rewrite. See `select-fire-rapidstrike-firmware-rp2040.ino` for the ported sketch.

## Pin assignments

Digital pins keep the same number as their Nano `D` pin, just prefixed `GP`. The two analog inputs move to the RP2040's dedicated ADC-capable pins (ADC0/ADC1), since the Nano's `A0`/`A1` don't exist as such on the RP2040.

| Signal | Nano (v1.0) | RP2040-Zero | Direction | Notes |
|---|---|---|---|---|
| Trigger switch | D2 | GP2 | input | `INPUT_PULLUP`, active-low |
| High-side FET gate | D3 | GP3 | output | |
| Cycle control switch | D4 | GP4 | input | `INPUT_PULLUP`, active-low |
| Low-side FET gate | D5 | GP5 | output | |
| Rotary switch — Safety | D6 | GP6 | input | `INPUT_PULLUP`, active-low |
| Rotary switch — Semi-auto | D7 | GP7 | input | `INPUT_PULLUP`, active-low |
| Rotary switch — Burst fire | D8 | GP8 | input | `INPUT_PULLUP`, active-low |
| Rotary switch — Full auto | D9 | GP9 | input | `INPUT_PULLUP`, active-low |
| Pusher/solenoid select jumper | D11 | GP11 | input | `INPUT_PULLUP`; open = pusher motor, jumpered = solenoid |
| Rate-of-fire potentiometer | A0 | GP26 (ADC0) | analog input | |
| Current sense | A1 | GP27 (ADC1) | analog input | |

GP16 (the RP2040-Zero's onboard WS2812 RGB LED) and GP0/GP1 (USB) aren't used by this firmware and are free for status LEDs or other additions.

## Power input

The RP2040-Zero can be powered via VIN/VSYS at either 5V or 3.3V — 5V (e.g. off the kit's existing buck converter output, or USB) is the common choice and is fine to use. This is independent of the ADC reference: VIN/VSYS feeds an onboard regulator that always produces a regulated 3.3V rail for the RP2040 chip, and the ADC reference (`ADC_AVDD`) is tied to that regulated 3.3V rail rather than to VIN. So regardless of whether the board is fed 5V or 3.3V, `analogRead()` always scales 0–1023 to 0–3.3V.

## Board-specific firmware changes (beyond pin numbers)

- **ADC reference voltage**: `ARDUINO_SUPPLY_VOLTAGE` (5.0 on the Nano) is renamed `BOARD_SUPPLY_VOLTAGE` and set to `3.3`. This is *not* the board's power input voltage (see above) — it's the RP2040's fixed ADC reference, and it feeds directly into the current-sense math (`analogReadingToVoltage()`). It must stay `3.3` regardless of what voltage powers the board, or current-sense/OCP readings will be off by ~52% (5.0/3.3). It must also match whatever reference the current-sense circuit actually presents to the ADC pin — see the hardware caveat below.
- **ADC resolution unchanged**: the arduino-pico core's `analogRead()` defaults to 10-bit (0–1023), same as the Nano's AVR ADC, so the existing `map()`/`/1024.0` scaling in `setRateOfFire()` and `analogReadingToVoltage()` didn't need to change. If you want the RP2040's full 12-bit ADC resolution, call `analogReadResolution(12)` in `setup()` and update those two spots to divide by 4096 instead.
- **Serial init no longer blocks**: the Nano's `initSerial()` had a `while(!Serial){}` wait, which is a no-op on the Nano's always-on hardware UART. The RP2040's `Serial` is native USB CDC and only becomes truthy once a host opens the port, so that same loop would hang the blaster indefinitely with no PC attached. The port drops the wait — `Serial.begin()` still runs so `Serial.print`/`println` calls work whenever something is listening, but firing no longer depends on it.

## Hardware caveats (if adapting the existing v1.0 PCB rather than a new board)

These aren't firmware changes — they're things to check before wiring an RP2040-Zero into a board designed for a 5V Nano:

- **Logic level is 3.3V, not 5V.** The RP2040-Zero's GPIOs are not 5V-tolerant. Anything driving a signal into an RP2040 GPIO (rotary switch common, trigger/cycle switches) is fine as long as it's just pulling to GND against the internal pullup, which all of these are. But double-check any circuitry that was assumed to be fed 5V logic.
- **FET gate drive is now 3.3V logic.** `HIGH_SIDE_PIN`/`LOW_SIDE_PIN` are driven directly at 3.3V instead of 5V. If the v1.0 board's gate drive circuitry (or the FETs themselves, if driven without a dedicated driver IC) assumes a 5V gate signal, verify it still fully turns on with 3.3V — this may need a logic-level FET, a 3.3V-compatible gate driver, or a level shifter to avoid the FET running in its linear region and overheating.
- **Current-sense scaling depends on the analog front-end's reference, not just the MCU's.** If the sense-resistor amplifier on the v1.0 board was scaled to present a 0–5V range to the Nano's ADC, feeding that directly into a 3.3V-max RP2040 ADC pin risks exceeding the input range. Confirm the analog front-end output swing is compatible with 3.3V before wiring it in, independent of the `BOARD_SUPPLY_VOLTAGE` firmware constant above.

## Dependencies

Same Arduino libraries as the Nano firmware (Library Manager): `JC_Button`, `CircularBuffer`, `arduino-timer`. Board support: install the [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico) core via Boards Manager ("Raspberry Pi Pico/RP2040"), then select the "Waveshare RP2040 Zero" board.
