#!/usr/bin/env python3
"""
Play a tune on a continuous-rotation servo running servo_tune.ino, record it,
and report how far each note landed from where it was aimed.

Usage:
    python tune.py COM13 [out_prefix]

A note is a rotation speed. The speed-to-pitch curve is measured fresh at the
start of every run, because a table measured earlier had already drifted: the
first attempt played its G about 35 Hz sharp, and its B came out as the C
before it, because the top of the old table sat where the curve goes flat and
a small error there is a whole semitone.
"""
import sys
import json
import time
import wave
import numpy as np
import serial

DELIM = b"===AUDIO_START==="
SR = 32000
BAND = (600.0, 1700.0)      # where the speed-tracking whine lives
MIN_PROMINENCE = 15.0       # peak over band median; below this, nothing is turning

QUARTER_MS = 600
# Happy Birthday, first phrase: G G A G C B, as (semitones above the lowest
# note, length in quarters). The second phrase needs a D above the range.
MELODY = [(0, 0.75), (0, 0.25), (2, 1), (0, 1), (5, 1), (4, 2)]
NAMES = ["G", "G#", "A", "A#", "B", "C", "C#", "D"]


def open_and_sync(port):
    for attempt in range(1, 4):
        ser = serial.Serial(port, 921600, timeout=1)
        ser.reset_input_buffer()
        ser.setDTR(False); ser.setRTS(True); time.sleep(0.1); ser.setRTS(False)
        buf = b""
        deadline = time.time() + 10
        while DELIM not in buf and time.time() < deadline:
            buf += ser.read(512)
        if DELIM in buf:
            return ser, bytearray(buf.partition(DELIM)[2].lstrip(b"\r\n"))
        ser.close()
        print(f"  no start marker (attempt {attempt}), retrying", flush=True)
        time.sleep(1)
    raise RuntimeError("board never sent its start marker")


def peak_hz(seg):
    """Strongest peak in BAND: (hz, rms, on_band_edge, prominence)."""
    seg = seg - seg.mean()
    n = 1 << 16
    spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg)), n))
    f = np.fft.rfftfreq(n, 1 / SR)
    m = (f >= BAND[0]) & (f <= BAND[1])
    sb = spec[m]
    i = int(np.argmax(sb))
    edge = i < 3 or i > len(sb) - 4
    prom = float(sb[i] / (np.median(sb) + 1e-9))
    return float(f[m][i]), float(np.sqrt(np.mean(seg ** 2))), edge, prom


def rising_chain(pts, min_step=5.0):
    """Longest run of points whose pitch climbs with speed.

    Isolated bad readings get skipped instead of setting the floor. The first
    version kept each point only if it beat the last one kept, so a single
    stray high reading near the deadband threw away everything genuinely lower
    and cut the range from about 6 semitones to 3.3.
    """
    n = len(pts)
    best = [1] * n
    prev = [-1] * n
    for i in range(n):
        for j in range(i):
            if pts[i][1] > pts[j][1] + min_step and best[j] + 1 > best[i]:
                best[i], prev[i] = best[j] + 1, j
    end = max(range(n), key=lambda i: (best[i], pts[i][1]))
    chain = []
    while end != -1:
        chain.append(pts[end])
        end = prev[end]
    return chain[::-1]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM13"
    prefix = sys.argv[2] if len(sys.argv) > 2 else "tune"

    ser, audio = open_and_sync(port)

    def listen(ms):
        end = time.time() + ms / 1000
        while time.time() < end:
            n = ser.in_waiting
            if n:
                audio.extend(ser.read(n))
            else:
                time.sleep(0.002)

    def samples(a, b):
        return np.frombuffer(bytes(audio[2 * a:2 * b]), dtype="<i2").astype(np.float64)

    # --- calibrate: sweep speed one way, measure the settled pitch at each ---
    print("Calibrating...", flush=True)
    listen(400)
    sweep = []
    for off in range(100, 501, 10):
        ser.write(f"W {1500 - off}\n".encode())
        listen(150)                               # let it get to speed
        a = len(audio) // 2
        listen(200)
        hz, rms, edge, prom = peak_hz(samples(a, len(audio) // 2))
        sweep.append((off, hz, rms, edge, prom))
    ser.write(b"W 1500\n")
    listen(500)

    usable = [(o, h) for o, h, r, e, p in sweep
              if r >= 100 and not e and p >= MIN_PROMINENCE]
    table = rising_chain(usable) if usable else []

    # Save the sweep before judging it, so a refusal leaves evidence behind.
    cal = {"sweep": [{"offset_us": o, "hz": round(h, 1), "rms": round(r, 1),
                      "edge": e, "prominence": round(p, 1)}
                     for o, h, r, e, p in sweep],
           "table": [{"offset_us": o, "hz": round(h, 1)} for o, h in table]}
    with open(f"{prefix}.cal.json", "w") as fh:
        json.dump(cal, fh, indent=1)

    print(f"  {'us':>4} {'Hz':>6} {'RMS':>5} {'prom':>6}")
    kept = {o for o, _ in table}
    for o, h, r, e, p in sweep:
        print(f"  {o:>4} {h:6.0f} {r:5.0f} {p:6.1f} {'<- used' if o in kept else ''}")

    if len(table) < 3:
        raise RuntimeError("no usable rising range in the sweep, see the table above")
    lo_hz, hi_hz = table[0][1], table[-1][1]
    span = 12 * np.log2(hi_hz / lo_hz)
    need = max(s for s, _ in MELODY)
    print(f"  playable {lo_hz:.0f}-{hi_hz:.0f} Hz ({span:.1f} semitones), "
          f"tune needs {need}", flush=True)
    if span < need:
        raise RuntimeError("this servo's range is too narrow for the tune today")

    hzs = [h for _, h in table]
    offs = [o for o, _ in table]

    def us_for(hz):
        return int(round(1500 - np.interp(hz, hzs, offs)))

    # Centre the tune inside the range, away from both ragged ends.
    base = lo_hz * 2 ** (((span - need) / 2) / 12)

    # --- play ---
    print("Playing...", flush=True)
    listen(500)
    marks = []
    prev = None
    for semi, q in MELODY:
        hz = base * 2 ** (semi / 12)
        ms = int(q * QUARTER_MS)
        if prev is not None and abs(hz - prev) < 1:
            # Repeated note: a short dip in pitch re-attacks it. A full stop
            # does not: the motor spins down into noise and the next note never
            # gets back to speed in time. Dipping before every step down was
            # also tried, on the theory that pitch depends on which way the
            # motor reaches a speed; the G it was meant to fix landed at 1037 Hz
            # with the dip and 1038 without, so it is not used.
            dip = max(lo_hz, hz * 2 ** (-2 / 12))
            ser.write(f"W {us_for(dip)}\n".encode()); listen(80); ms -= 80
        marks.append((len(audio) // 2, hz, ms, semi))
        ser.write(f"W {us_for(hz)}\n".encode())
        listen(ms)
        prev = hz
    ser.write(b"W 1500\n")
    listen(800)
    ser.close()

    n = len(audio) // 2
    x = samples(0, n)
    with wave.open(f"{prefix}.wav", "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes(x.astype("<i2").tobytes())
    cal["base_hz"] = round(base, 1)
    with open(f"{prefix}.cal.json", "w") as fh:
        json.dump(cal, fh, indent=1)
    print(f"Saved {prefix}.wav ({len(x)/SR:.1f}s, peak {int(np.max(np.abs(x)))}/32767) "
          f"and {prefix}.cal.json")

    print()
    print(f"{'note':>5} {'aimed':>9} {'played':>9} {'off by':>9}")
    offs_c = []
    for start, hz, ms, semi in marks:
        a = start + int(ms * 0.5 / 1000 * SR)
        b = start + int(ms / 1000 * SR)
        got = peak_hz(x[a:b])[0]
        cents = 1200 * np.log2(got / hz)
        offs_c.append(cents)
        print(f"{NAMES[semi]:>5} {hz:8.0f}Hz {got:8.0f}Hz {cents:+8.0f}c")
    good = sum(1 for c in offs_c if abs(c) <= 30)
    print()
    print(f"{good} of {len(offs_c)} notes within 30 cents (100 cents = one semitone).")


if __name__ == "__main__":
    main()
