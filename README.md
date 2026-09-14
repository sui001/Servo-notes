# Servo + INMP441 note-scan test

## Wiring
- INMP441: VDD→3V3, GND→GND, L/R→GND (left channel), WS→GPIO5, SCK→GPIO4, SD→GPIO6
- Servo signal→GPIO7. Power the servo from its own 5V supply (not the S3's
  regulator) and share ground with the ESP32. Avoid strapping pins 0/3/45/46.

## Flashing
1. Arduino IDE, board = "ESP32S3 Dev Module" (or the Supermini variant if listed).
2. Install `ESP32Servo` library.
3. In `servo_mic_test.ino`, set `TEST_POSITIONAL` / `TEST_CONTINUOUS` for
   whichever unit is wired up right now — only one at a time, since they're
   physically different servos.
4. Flash, then run `analyze.py <port> <name>` on the computer — it waits
   for the plan, then records the whole run and saves `<name>.wav` +
   `<name>.csv`.

## What's tested
**Positional (180°) servo:** 9 target distances (5°–150° from home) ×
4 command-rate profiles (jump, and 20/10/5 ms per degree) × 3 repeats
= 108 events.

**Continuous-rotation servo:** 7 speed levels × both directions × 3
repeats = 42 events.

Note: a 180° servo can't be turned into continuous-rotation by code —
that needs the actual continuous-rotation part (different gearing, no
end-stop). So "test both" means having one of each in hand, run as two
separate flashes.

## Output
`analyze.py` gives a CSV of every event's detected frequency, nearest
equal-tempered note (A4=440), and cents off. Low `confidence` rows mean
the segment was noise-like rather than a clear pitch — expected for some
combos, and itself useful information (that region isn't a usable "note").

## Time estimate
- Per positional unit: 108 events × ~1.6s ≈ 3 min recording.
- Per continuous unit: 42 events × ~1.9s ≈ 1.5 min recording.
- If you test 9g positional + one bigger positional + 1–2 continuous
  units: ~4 flashes/swaps, ~10 min of actual recording total.
- Add setup (quiet room, mic placement, re-flashing per unit, checking
  each WAV isn't clipping or full of mains hum): realistically
  **45–60 minutes** for the first full pass across 3–4 servos.

Second pass will be faster once the rig and plan are known-good.
