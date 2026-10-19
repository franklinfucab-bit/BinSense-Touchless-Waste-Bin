# Smart Garbage Bin - Development Log

EE 474 SUM26, Group 22. Shawn Yin and Franklin Fu.
Last updated: August 21, 2026 (evening — AM2320 I2C driver landed).

This log is written for a person or agent picking the project up cold. It records
the current state, the reasoning behind choices that are not obvious from the
code, and the one problem still open.

---

## 1. Hardware and toolchain

- ESP32-S3-WROOM-1 (ESP32S3 Dev Module), Arduino IDE 2.3.7, Arduino-ESP32 3.3.10
- Serial monitor at 115200
- Sensors: ASAIR AM2320 (was mislabeled DHT11), HC-SR04, HC-SR501 PIR, LDR divider
- Actuator: 28BYJ-48 stepper through a ULN2003 module, external MB-102 supply
- UI: 1602 I2C LCD at address 0x27, passive buzzer

Two hardware rules that have already caused trouble:

- ESP32-S3 GPIOs are **not** 5 V tolerant. DHT11 runs from 3V3. If the HC-SR04
  echo output is 5 V, it needs a divider before GPIO 6.
- The stepper draws from the external supply, not the ESP32 3V3 rail. Grounds
  are shared.

## 2. Where files live, and why

Working copy: `C:\Users\Frank\Documents\EE474_Smart_Bin`

The original project folder is on Google Drive at
`G:\My Drive\EE474 Embedded system\Final Project`. **Do not write `.ino` files
there.** Writes to that path came back UTF-16/mojibake-corrupted several times.
The Drive folder is fine as a read-only reference and for the proposal PDF.

```
module_tests/          copies of the four passed component tests + proposal PDF
firmware/pins.h        canonical pin map (reference copy)
firmware/stage2_pir_lid/
firmware/stage3_sensors_ui/
firmware/smart_garbage_bin/     full baseline
firmware/dht_diag/              obsolete: assumed a DHT11, see section 6
firmware/am2320_diag/           I2C diagnostic for the real part (AM2320)
firmware/ldr_cal/               LDR calibration bench tool
```

Each sketch folder carries its own identical `pins.h` because the Arduino IDE
compiles one sketch directory at a time and will not reach up a level. If you
change a pin, change every copy. `firmware/pins.h` is the master to copy from.

The approved implementation plan lives at
`c:\Users\Frank\.cursor\plans\smart_bin_integration_78bacd11.plan.md`.
It is a reference document and should not be edited.

## 3. Pin map (current)

| GPIO | Use | Notes |
|---|---|---|
| 1 | LDR analog | lid-closure feedback, not yet hardware-verified |
| 2 | Passive buzzer | LEDC tone, C5 = 523 Hz |
| 5 | HC-SR501 PIR | |
| 6 / 7 | HC-SR04 echo / trigger | |
| 8 / 9 | I2C SDA / SCL | LCD 0x27 **and** AM2320 0x5C. Clock 100 kHz. |
| 10 | unused | freed when the "DHT11" moved to I2C |
| 15-18 | Stepper IN1-IN4 | ULN2003 |

**GPIO 4 is retired.** It was the LED in the original PIR component test, then
became DHT11 data, and DHT never worked there. It was moved to GPIO 10 on
August 21 while chasing the open issue in section 6. Nothing should be wired to
GPIO 4 now.

### Sharing one I2C bus between the LCD and the AM2320

Both devices sit on GPIO 8 / 9. This works because of three things, and breaking
any of them will produce intermittent faults that are painful to diagnose:

1. **`Wire.begin()` and `Wire.setClock()` happen once in `setup()`, before any
   task is created.** No task re-initialises the bus.
2. **`i2cMutex` is held across a whole logical transaction, not per byte.** In
   `am2320Task` the mutex wraps the entire `readAm2320()` call, which internally
   is wake / 2 ms / command / 2 ms / read. If the LCD were allowed to write
   between the wake-up and the command, the sensor would drop back to sleep and
   the read would fail. `uiTask` takes the same mutex around its LCD writes.
3. **The clock is 100 kHz.** That is the AM2320's stated *maximum*, not a
   recommendation. The LCD is happy far above this, so the sensor sets the limit.

**If `am2320_diag` reports CRC failures rather than silence, try 50 kHz first.**
Change `I2C_CLOCK_HZ` in `pins.h` and copy it out. Running a two-device bus at a
part's absolute maximum with breadboard wiring is the usual cause of
intermittent CRC errors, and halving the clock costs nothing here because the
sensor is only read every 2 s.

`am2320_read.h` is duplicated into every sketch folder for the same reason
`pins.h` is. Keep all copies identical; `firmware/am2320_read.h` is the master.

### Observed August 21: LCD turns to garbage mid-run, sensor keeps working

**Symptom.** Both devices come up fine. After some minutes the LCD shows garbage
characters and never recovers, while AM2320 temperature readings stay correct.

**Why the asymmetry, and why it identifies the fault.** The AM2320 is stateless:
every read is a fresh wake / command / response, so a corrupted transaction just
fails once and the next one succeeds. The 1602 is an HD44780 driven through a
PCF8574 in **4-bit mode**, where each byte is sent as two nibbles plus an enable
pulse. Lose or gain a single nibble and the controller is permanently one nibble
out of phase, so every later byte is reassembled from the wrong halves — garbage
forever, until it is re-initialised. So this symptom pair does **not** mean the
bus is broken; it means the bus glitched once and only one of the two devices
carries state across the glitch.

**Mitigations now in the code.** `uiTask` re-initialises the LCD every
`LCD_REINIT_MS` (5 s), so a desync heals by itself instead of persisting until a
power cycle. All LCD strings are also padded to the full 16 columns, because a
short write leaves trailing columns untouched and garbage would otherwise sit
there. `stage3_sensors_ui` does the same. This makes the display robust, but it
treats the symptom.

**Root cause CONFIRMED on hardware: stepper noise.** The garbling correlates
with lid opening and closing, and it clears on the next LCD refresh, roughly
100 ms (`UI_PERIOD_MS`). Two useful conclusions from that:

- It is **not** a 4-bit phase desync after all. A desync would persist until
  re-initialisation. Since a single padded rewrite fixes it, the controller
  stays in sync and only the written data was corrupted. Less serious than
  feared, and the padding is what heals it.
- Because it tracks lid motion, it is the 28BYJ-48 switching inductive loads
  next to unshielded breadboard I2C wiring, not an I2C protocol problem.

`I2C_CLOCK_HZ` was dropped to **50000** in response. Slower edges tolerate the
motor's switching noise far better, and the sensor is read only every 2 s.

**The remaining fix is electrical, in priority order:**

1. **Grounding.** The stepper's return current must not share a path with the
   I2C ground. Bring the external supply ground and the ESP32 ground together at
   a single point rather than daisy-chaining through the breadboard rail.
2. **Decoupling.** 100 nF ceramic right at the LCD module's VCC/GND pins, and a
   bulk electrolytic (100 uF or more) across the ULN2003 supply input. The coil
   current steps cause supply dips.
3. **Routing.** Physically separate the four motor wires from SDA/SCL. Do not
   bundle or run them parallel; cross at right angles if they must cross.
4. **Shorter I2C leads**, and if it still misbehaves, add 2.2k-3.3k pull-ups in
   parallel with the module's existing ones for faster edges.

**Optional software mitigation if it still shows.** Suppress LCD writes while
the stepper is actively stepping: write once on entering `OPENING`/`CLOSING` so
the state is visible, then hold until motion completes. That keeps I2C traffic
off the bus during exactly the window when it is noisy. Not implemented, because
the artifact is cosmetic and self-clearing.

Stepper constants: 2048 steps per revolution, 512 steps = 90 degrees, 8 RPM.
The Arduino `Stepper` constructor takes pins in the order `(IN1, IN3, IN2, IN4)`.
Every sketch preserves that ordering from the passed `stepper_90_test`.

## 4. Build order

Flash these in sequence and grant PASS only from hardware evidence.

1. `firmware/stage2_pir_lid` - PIR opens the lid, closes after 10 s idle.
   No extra libraries. **Confirmed working on hardware.**
2. `firmware/stage3_sensors_ui` - adds fill sensing, buzzer, LCD, and AM2320.
   Needs `LiquidCrystal_I2C`. AM2320 shares I2C with the LCD.
3. `firmware/smart_garbage_bin` - full baseline. Needs `LiquidCrystal_I2C`.

`firmware/am2320_diag`, `firmware/ldr_cal`, and the now-obsolete
`firmware/dht_diag` are not part of the sequence. They are bench tools.

## 5. Design decisions worth knowing

**The stepper cannot use `Stepper.step(512)` in the real firmware.** That call
blocks for the entire 90 degree move, which is impossible inside a 100 Hz control
task. `smart_garbage_bin` drives the ULN2003 coils directly, a few steps per
100 Hz tick, using a phase table that matches the passed test's coil order. The
two staging sketches still use the blocking library call, which is acceptable
there.

**PIR opens immediately, but closes slowly.** Motion opens the lid the moment
GPIO 5 reads HIGH. Closing waits until the PIR has been LOW continuously for
`PIR_HOLD_OPEN_MS` (10 s), and any new motion restarts that timer. This came
from a real complaint: the sensor was too twitchy and the lid was slamming. An
earlier version only reacted to rising edges, which made the lid miss waves
entirely. Do not go back to edge-only detection.

**Temperature/humidity is an ASAIR AM2320 on I2C, not a DHT11.** Driver is
`am2320_read.h`: wake (expected NACK), then function 0x03 read of 4 registers,
CRC-16 check, values in tenths. Address 0x5C. In `smart_garbage_bin` this runs
in `am2320Task` on core 0 and shares the bus with the LCD through `i2cMutex`.
`AM2320_ENABLED` is 1.

**A failed AM2320 read keeps the last good value.** It does not blank the LCD.
Only a sustained absence of valid data (`DHT_STALE_MS`, 10 s, after at least one
good read) escalates to FAULT-SAFE E03.

**Homing was deliberately relaxed.** The system starts in `CLOSED` rather than
`HOMING`, a homing timeout falls back to `CLOSED` instead of FAULT-SAFE, and PIR
motion during homing opens the lid. Earlier, an uncalibrated LDR meant homing
never completed, so the lid appeared dead. **That root cause is now fixed and
understood — see section 7b.** The blocker for restoring the proposal's stricter
homing is therefore gone; it is step 3 in section 10, pending one hardware run
to confirm homing completes.

**The `t+20` and `h+30` offsets in the original `dht11_test` were display
fudges, not calibration.** They are intentionally not carried into any firmware.

## 6. RESOLVED IN CODE: the "DHT11" is an ASAIR AM2320

**Status: firmware now talks I2C AM2320. Hardware still needs a rewire + PASS.**

**The part is an ASAIR AM2320 (marking 2320), not a DHT11.** This single fact
explains every failed attempt below, and there are two independent reasons the
old code could never have worked:

1. **The pinout is different.** DHT11 is `VDD / DATA / NC / GND`. AM2320 is
   `VDD / SDA / GND / SCL` — pins 3 and 4 swap roles. Wiring it as a DHT11 puts
   ground on the SCL pin and leaves the real ground (pin 3) floating, so the
   sensor never had a valid ground. That is a completely silent failure and
   matches the observed total non-response.
2. **The data format is different.** Even over its single-wire mode the AM2320
   uses the DHT22 format: 16-bit humidity and 16-bit temperature in tenths.
   `dht11_read.h` decodes the DHT11 format: 8-bit integers. The checksum is a
   plain byte sum so it could even pass while the decoded values were nonsense.

This is why none of the four fixes in the history below changed anything. They
were all refinements of the wrong protocol on miswired pins. The sensor was
probably never damaged and GPIO 4 was probably never damaged either.

**Plan: switch to I2C and abandon the single-wire path entirely.** The AM2320
speaks I2C at address `0x5C`, and this board already has a working I2C bus at
GPIO 8 / GPIO 9 with the LCD proven at `0x27`. Going I2C removes every source of
pain that consumed days here: no bit-banged timing, no `pulseIn()` interference,
no `noInterrupts()` window, no sensitivity to FreeRTOS scheduling. It also frees
GPIO 10. For the report it is a better story too — one bus, two addresses.

**Next action:** flash `firmware/am2320_diag`. It scans the bus and then does a
full AM2320 read with the required wake-up, reporting which step failed. One
important subtlety it prints for you: **the AM2320 sleeps and normally does not
answer an I2C scan**, so `0x5C` missing from the scan list proves nothing — only
the read test counts.

**Current gate.** `pins.h` defines `AM2320_ENABLED 1`, and the firmware compiles
the I2C path in. The old `DHT_ENABLED` flag and `PIN_DHT11` no longer exist.
`dht11_read.h` and `firmware/dht_diag` are dead weight kept only as history.

**Why an unresponsive AM2320 still cannot block anything.** `FAULT E03` requires
`sawValidDht` to be true first, i.e. at least one successful read must have
happened at some point in that power cycle. If the sensor never answers, the LCD
simply keeps showing no temperature and the state machine is untouched. So the
sensor can be enabled safely before it is proven on hardware — which is the
situation right now.

Note the internal symbols are still named `dht*` (`sawValidDht`, `lastDhtOkMs`,
`DHT_STALE_MS`, `SensorData::dhtValid`). Cosmetic only, but it makes the code
read as if a DHT11 is present. Worth a rename pass before the report.

Consequences until the AM2320 passes on hardware:

- Verification rows **N9** and **F3** cannot be run. Everything else can.
- Novel-hardware minimum is still met without it: HC-SR04 and HC-SR501 are two
  peripherals not used in earlier labs.

**Symptom when the one-wire code was last used.** LCD line 2 showed
`T=--.-C H=--`. Serial printed `DHT bad read` every 2 s. The lid, PIR, stepper,
and boot sequence were all fine.

**History.** `module_tests/dht11_test` was believed to have passed on GPIO 4 with
the Adafruit library, which is what anchored the wrong assumption for so long.

**History below is kept for the report — it is a record of chasing a wiring and
protocol mismatch, and every step of it was reasonable given the wrong
assumption about which part was on the breadboard.**

**What has been tried, in order:**

1. Separated the DHT read from the HC-SR04 `pulseIn()` so they never run in the
   same pass. No change.
2. Replaced the Adafruit library with the local `dht11_read.h` driver, and in
   the full firmware moved it to a dedicated task. No change.
3. Added the required 4.7k-10k pull-up from DATA to 3V3. No change.
4. Moved DATA from GPIO 4 to GPIO 10, suspecting a damaged pin. No change.

**Scope observation and why it matters.** With the probe on the DHT data line,
the user measured roughly **250 ms high, 5 ms low, repeating**. The firmware
emits a **20 ms low pulse once every 2 s**. Neither the width nor the period
matches, which strongly suggests the probe is not on the pin being driven by the
DHT code. Note that `stage3_sensors_ui` refreshes the LCD on a 250 ms cycle, and
each refresh is a few milliseconds of I2C traffic on GPIO 8 and 9 - that is a
much better match for what was measured.

**Protocol reminder.** DHT11 is a single-wire proprietary protocol, not I2C. The
only I2C on this board is the LCD. Expected frame: 20 ms low from the MCU, a
20-40 us release, an 80 us low / 80 us high presence pulse from the sensor, then
40 bits where each bit is 50 us low followed by a high of about 26 us for a 0 or
70 us for a 1. The whole sensor reply is only about 4 ms, so it is invisible at
slow sweep speeds.

**Next step when the DHT is picked back up.** Flash `firmware/dht_diag`. It runs
nothing but the DHT, and it does not read `DHT_ENABLED` — it is a standalone
diagnostic and always runs. On boot
it drives the pin as a 1 Hz square wave for 10 s so the probe placement can be
confirmed beyond doubt, then reports which step of the handshake failed along
with measured pulse widths. `NO_ANSWER` means the pin works but the sensor is
silent, which points at power, ground, or the part. `BAD_CHECKSUM` or a bit
timeout means the sensor is talking and only the decoding is off. Change
`DHT_PIN` at the top of that sketch to compare GPIO 10 against GPIO 4.

## 7. Not yet verified on hardware

- LDR on GPIO 1 is now **calibrated** — see section 7b. Two caveats remain
  there: where the lid physically stops, and ambient light dependence.
- 1602 LCD in this project. The pin choice and 0x27 address come from Lab 3.
- The full `smart_garbage_bin` baseline end to end.
- BLE and IR are deliberately not implemented. The proposal defers them until
  core V&V passes.

## 7a. How to calibrate the LDR

Flash `firmware/ldr_cal`. It is standalone, needs no extra libraries, and is
driven from the serial monitor at 115200.

| Key | Action |
|---|---|
| `k` | capture 200 samples with the lid **closed** |
| `n` | capture 200 samples with the lid **open** |
| `r` | print recommended constants |
| `t` | live-test those constants while you move the lid |
| `s` | stream raw and filtered readings |
| `o` / `c` | step the lid 90 degrees open / closed |
| `x` | clear captures |

Procedure: close the lid, press `k`. Open the lid, press `n`. Press `r`. It
prints three lines ready to paste into `smart_garbage_bin.ino`, then press `t`
and move the lid by hand to confirm it declares CLOSED and OPEN where it should.

What it is doing, so the numbers can be argued for in the report: it takes the
mean of each state, puts the setpoint midway between them, and sets hysteresis
to a quarter of the separation. Polarity is inferred — if the closed mean reads
lower than the open mean then `LDR_CLOSED_MEANS_DARK` is `true`. In this
codebase "dark" means **low ADC counts**, which depends on how the divider is
wired, so let the tool decide rather than reasoning about it.

Two guardrails it applies. Hysteresis is capped at just under half the
separation, because the setpoint sits midway between the means and a wider band
would push the edge past the mean, leaving the state stuck forever. And if the
two ranges overlap at all, it says so and refuses to pretend a threshold exists
— that means the optics need fixing (shade the LDR, move it nearer the lid
edge, or change the divider resistor) before any number will help.

The tool deliberately leaves the ADC at the core defaults (12-bit, 11 dB)
because `smart_garbage_bin.ino` calls `analogRead()` without configuring it. If
you ever configure the ADC in one, do it in both or the calibration will not
transfer.

## 7b. LDR calibration result (August 21, measured)

Captured with `firmware/ldr_cal`, 200 samples per state:

| State | mean | sd | min | max |
|---|---|---|---|---|
| CLOSED | 1053.7 | 7.8 | 1041 | 1068 |
| OPEN | 3276.7 | 1.4 | 3273 | 3283 |

Separation is 2223 counts against a worst-case sd of 7.8 — the two states are
about 285 sd apart and do not overlap. Applied to `smart_garbage_bin.ino`:

```c
constexpr int LDR_CLOSED_SETPOINT = 2165;
constexpr int LDR_HYSTERESIS = 556;
constexpr bool LDR_CLOSED_MEANS_DARK = true;   // unchanged, confirmed
```

**The closed-loop target is now expressed as a percentage of that span, not as
raw counts.** August 21 change:

```c
constexpr int LDR_RAW_CLOSED = 1054;        // measured, lid shut
constexpr int LDR_RAW_OPEN   = 3277;        // measured, lid open
constexpr int LDR_CLOSE_TARGET_PCT = 45;    // where the lid is declared shut
constexpr int LDR_HYSTERESIS = 556;
```

Everything else is derived: `LDR_CLOSED_THRESHOLD = 1054 + 2223*45/100 = 2054`,
`LDR_OPEN_THRESHOLD = 2610`, and polarity comes from which endpoint is larger
rather than a separate `LDR_CLOSED_MEANS_DARK` flag that could contradict them.
Three `static_assert`s fail the build if the percentage or hysteresis would put
either band outside the measured range, which is the failure mode that made the
lid look dead in the first place.

**`LDR_CLOSE_TARGET_PCT` is the knob for where the lid physically stops.** Lower
drives it further shut, higher stops it earlier. The previous raw setting of
1609 was equivalent to 25%, so **45% stops the lid earlier — less far closed —
than before.** Reference values against the measured data:

| Target | closed thresh | open thresh | margin to open reading |
|---|---|---|---|
| 25% | 1609 | 2165 | 1108 |
| 35% | 1832 | 2388 | 885 |
| 45% (current) | 2054 | 2610 | 663 |
| 55% | 2276 | 2832 | 441 |

All measured samples, including both extremes, still classify correctly at 45%.
`firmware/ldr_cal` was updated to print this same parameter form so the tool and
the firmware cannot drift apart.

**This confirms why the lid used to look dead.** The old guess put the closed
band at `<= 1000`, but the real closed readings are 1041-1068 — above the closed
band and below the open band, so they landed in the dead zone between the two.
`gLdrClosed` could never become true, so homing never completed. Open was
detected fine. It was not a mechanical or stepper fault, and not a damaged pin;
the threshold was simply on the wrong side of reality.

**Caveat 1: where the lid physically stops.** Both `HOMING` and `CLOSING` stop
stepping the instant `gLdrClosed` goes true, so the closed threshold decides the
resting position. The band edge of 1609 sits 25% of the way from the closed
reading toward the open reading. Whether that is 2 degrees or 20 degrees short of
true closure depends on the light-versus-angle curve, which is nonlinear and has
not been measured. Check it on hardware: if the lid visibly stops short, tighten
the closed band. Verified alternatives, all of which keep both bands reachable:

| Option | setpoint | hysteresis | closed band | margin above closed max | trips at |
|---|---|---|---|---|---|
| current | 2165 | 556 | <= 1609 | 541 counts (69 sd) | 25% from closed |
| tighter | 2000 | 750 | <= 1250 | 182 counts (23 sd) | 8.8% from closed |
| tightest | 1900 | 800 | <= 1100 | 32 counts (4 sd) | 2.1% from closed |

Do not go past "tightest" — 4 sd of margin is already thin, and ambient light
drift is far larger than sd.

**Caveat 2: this is an ambient light sensor.** The numbers above are only valid
for the lighting they were captured in, and the open reading of 3277 is close to
the 4095 ADC ceiling, which suggests a bright room. In a darker room the open
reading will fall, and if it falls below 2721 the lid stops reporting open; if it
fell below 1609 the firmware would falsely declare the lid closed. Before the
demo, re-run `ldr_cal` under the actual demo lighting, or at minimum stream with
`s` and confirm the open reading stays above 2721 with the room lights off.
Shading the LDR so it sees lid position rather than room light is the real fix.

## 7c. The two hardware timers and the 64 Hz input task

Added August 21. This one subsystem covers three separate rubric minimums, which
is why it looks disproportionately documented.

| Timer | Core | Rate | Releases | Purpose |
|---|---|---|---|---|
| `controlTimer` | 1 | 100 Hz | `controlTimerSemaphore` | lid control, LDR feedback, stepping |
| `inputTimer` | 0 | 64 Hz exactly | `inputTimerSemaphore` | `inputTask`: sample and debounce physical inputs |

**Why 64 Hz had to come from a hardware timer.** The rubric asks for a task at
32, 64, or 128 Hz. Those periods are 31.25 ms, 15.625 ms, and 7.8125 ms — none of
them a whole number of milliseconds. The FreeRTOS tick here is 1 ms, so
`vTaskDelay`/`vTaskDelayUntil` cannot express any of them; the closest to 64 Hz
is 15 ms (66.7 Hz) or 16 ms (62.5 Hz). A hardware timer has a 1 us tick, so
`1000000 / 64 = 15625 us` is exact. This is almost certainly why the course
specified those three numbers.

**Why `inputTimer` is started inside `inputTask` and not in `setup()`.** An ESP32
timer interrupt is installed on whichever core calls `timerAttachInterrupt()`.
`setup()` runs in `loopTask`, which Arduino pins to core 1, so starting the timer
there would put its ISR on core 1 next to the lid timer. `inputTask` is pinned to
core 0 and starts its own timer, so the ISR lands on core 0 and each core owns
one timer driving one real-time task. **If you ever move those three lines back
into `setup()`, the per-core split silently disappears.**

**PIR debouncing.** `inputTask` requires `PIR_CONFIRM_SAMPLES` (3) consecutive
disagreeing samples before it accepts a level change — about 47 ms at 64 Hz,
short enough that opening still feels instant. The debounced result goes to
`gPirStable` under `sharedMutex`; `sensorTask` copies it into
`SensorData::pirHigh`, so the state machine is unchanged and still reacts to a
level, not an edge (see section 5 — do not go back to edge-only detection).

This also closes the gap between the code and the proposal, which claimed
"IR/PIR events are filtered" while the PIR was in fact a bare `digitalRead`.

**Debounce summary for all physical inputs**, which the rubric asks you to
explain input by input:

| Input | Method |
|---|---|
| PIR (GPIO 5) | 3 consecutive samples at 64 Hz in `inputTask` |
| LDR (GPIO 1) | 5-point moving average, then 5 consecutive in-band samples |
| HC-SR04 (GPIO 6) | 3 consecutive near readings to declare FULL, plus 10/12 cm hysteresis |
| buttons | not fitted; `inputTask` is where they belong |

## 7d. Timing evidence: how to produce the numbers the proposal promised

The proposal justifies the "High performance" criterion with *"measured
deadline/jitter results under synthetic CPU load"*. Until August 21 the firmware
recorded only a count of ticks whose work exceeded 8 ms — no jitter, no
worst-case execution time, and no way to add load. All three now exist.

**Serial commands** (115200):

| Command | Effect |
|---|---|
| `stats` | print the timing report |
| `statsreset` | clear the counters and start a fresh window |
| `load on` / `load off` | enable/disable synthetic CPU load on core 1 |
| `reset` | leave FAULT-SAFE and re-home (unchanged) |
| `help` | list commands |

**What is measured, and the distinction that matters.** Two different numbers get
confused in reports, so be precise when presenting them:

- **period** is measured start-of-work to start-of-work. Nominal is 10000 us for
  the control task and 15625 us for the input task. It therefore captures timer
  jitter *and* release latency, i.e. how late the task actually started.
  **Jitter** is reported as the worst absolute deviation from nominal.
- **exec** is the duration of the work itself, i.e. worst-case execution time.
  This is what gets compared against the deadline budget.

A task can have perfect exec times and terrible jitter, or the reverse. The
rubric's phrase "deadline" concerns exec versus budget; "jitter" concerns period.

**How to run it for the report:**

1. Boot, let it settle, then `statsreset`.
2. Exercise the lid a few times so the control task does real stepping work.
3. `stats` — this is the unloaded baseline.
4. `load on`, wait 30 s or so, exercise the lid again, `stats` — this is the
   loaded case. Compare jitter and exec max against the baseline.
5. `load off`.

**Why the load task is built the way it is.** It is pinned to core 1 at priority
1, below the 100 Hz control task (5) and the 50 Hz safety task (4), so it can
only absorb CPU those two leave behind and can never starve them — the point is
to load the system, not to break it. It spins for 8 ms and then yields one tick,
because a pure busy loop would block core 1's idle task and trip the FreeRTOS
task watchdog, which produces a crash dump instead of a measurement.

**Why the instrumentation takes no mutex.** Each `TimingStats` instance has
exactly one writer task, readers only read, and aligned 32-bit accesses are
atomic on the ESP32-S3. Locking inside the measured window would add contention
to the very interval being measured, so the numbers would describe the
instrumentation rather than the system.

Note `gDeadlineMissCount` (persisted in `Preferences`, lifetime) and
`TimingStats::overrunCount` (since last `statsreset`) count the same event over
different windows. Do not present them as independent results.

## 8. Architecture of the full firmware

Core 1 owns real time. A hardware timer ISR does nothing but give a binary
semaphore at 100 Hz; the lid control task takes it, samples the LDR, and advances
the stepper a few steps. A 50 Hz safety task runs the state machine over
`HOMING`, `CLOSED`, `OPENING`, `HOLD_OPEN`, `CLOSING`, `FULL`, `FAULT_SAFE`.

Core 0 owns services: the sensor task (PIR, HC-SR04, publishes a `SensorData`
snapshot), the DHT task, and the UI task (LCD, buzzer, LittleFS logging).

Three queues carry data between them, as the proposal requires:
`sensorDataQueue`, `controlCommandQueue`, `systemEventQueue`. Shared globals are
guarded by `sharedMutex`. Counters for openings, timeouts, and deadline misses
persist through `Preferences`. The event log is CSV at `/bin_log.csv` on
LittleFS.

Fault codes shown on the LCD as `FAULT Enn`:

| Code | Meaning |
|---|---|
| E01 | home timeout |
| E02 | close timeout |
| E03 | stale DHT data |
| E04 | control loop deadline misses |

Typing `reset` into the serial monitor clears FAULT-SAFE and re-homes.

## 9. Verification table

Thresholds referenced below live in `smart_garbage_bin.ino`.

**Normal**

| # | Stimulus | Expected |
|---|---|---|
| N1 | Power-up | reaches `CLOSED`, coils idle |
| N2 | Wave at PIR | opens ~90 degrees immediately, `EVENT motion` |
| N3 | Stay in front | remains `HOLD` |
| N4 | Walk away | closes 10 s after last motion |
| N5 | Wave during close | reverses to `OPENING` |
| N6 | Object under 10 cm for 3 samples | `FULL`, buzzer on |
| N7 | Object beyond 12 cm | back to `CLOSED`, buzzer off |
| N8 | Wave while `FULL` | opens so the bin can be emptied |
| N9 | DHT connected | line 2 shows T and RH — **blocked, DHT gated off** |
| N10 | Lid open | ranging pauses, resumes when closed |

**Boundary**

| # | Stimulus | Expected |
|---|---|---|
| B1 | LDR just outside closed band | does not declare closed |
| B2 | LDR inside band, 5 samples | declares closed, stops stepping |
| B3 | Distance between 10 and 12 cm | no state chatter |
| B4 | Empty vs full | fill percent near 0 vs 100 |

**Fault**

| # | Stimulus | Expected |
|---|---|---|
| F1 | Block the lid while homing | stops, `FAULT E01` |
| F2 | Block the lid while closing | stops, `FAULT E02` |
| F3 | Unplug DHT after a good read, wait 10 s | `FAULT E03` — **blocked, DHT gated off** |
| F4 | 10 consecutive control overruns | `FAULT E04` |
| F5 | Any fault | coils off, logged to LittleFS |
| F6 | Type `reset` | leaves fault, re-homes |

## 10. Suggested order of work

DHT11 is deliberately no longer first. It is gated off (section 6) so it cannot
block anything below it.

1. ~~Calibrate the LDR~~ **done August 21, see section 7b.** Constants are in
   `smart_garbage_bin.ino`.
2. Flash `smart_garbage_bin` and confirm homing now actually completes and the
   lid closes fully. This is the payoff test for step 1 — it should fix the
   "lid appears dead" behaviour. If the lid stops short of closed, take the
   tighter thresholds from the table in section 7b.
3. Restore strict homing once step 2 passes: start in `HOMING` instead of
   `CLOSED`, make a homing timeout raise `FAULT E01` instead of falling back to
   `CLOSED`, and reconsider whether PIR motion should open during homing. See
   section 5 for why each was relaxed.
4. Run the verification table against `smart_garbage_bin`, skipping N9 and F3.
   Rows B1 and B2 are now meaningful for the first time: the LDR band edges are
   1609 and 2721, so "just outside the closed band" is a real, known number.
5. ~~Measure 100 Hz jitter under synthetic CPU load~~ **instrumentation done,
   see section 7d.** What remains is running it and pasting the numbers into the
   report. Still needs a hardware run.
6. Come back to the DHT11 with `dht_diag`. If it turns out to be a dead part,
   the flag stays at 0 and the report explains the two skipped rows.
7. Only then consider BLE and IR.
