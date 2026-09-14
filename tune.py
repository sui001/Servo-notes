#!/usr/bin/env python3
"""
Play a tune on a continuous-rotation servo running servo_tune.ino, record it,
and report how far each note landed from where it was aimed.

Usage:
    python tune.py COM13 [out_prefix]

A note is a rotation speed. The table below is one servo's measured
speed-to-pitch curve (conttest.wav, turning one way). It is calibration for
that unit, not a property of servos in general, so a different servo needs its
own run of servo_mic_test in continuous mode and a new table.
"""
import sys
import time
import wave
import numpy as np
import serial

DELIM = b"===AUDIO_START==="

# Offset below 1500 us -> strongest whine peak under 2 kHz, from conttest.wav.
# This direction because it rose steadily all the way to 500 us; the other
# direction flattened past 300 us and dipped at 400.
CAL = [(150, 898.4), (200, 1074.2), (300, 1152.3), (400, 1238.3), (500, 1285.2)]

BASE = 950.0  # the tune's lowest note, inside the playable range
QUARTER_MS = 500

# Happy Birthday, first phrase: G G A G C B, as semitones above BASE.
# The second phrase needs a D, which is above what this servo reaches.
MELODY = [(0, 375), (0, 125), (2, 500), (0, 500), (5, 500), (4, 1000)]


def us_for(hz):
    hzs = [h for _, h in CAL]
    offs = [o for o, _ in CAL]
    if not hzs[0] <= hz <= hzs[-1]:
        raise ValueError(f"{hz:.0f} Hz is outside this servo's range "
                         f"{hzs[0]:.0f}-{hzs[-1]:.0f} Hz")
    return int(round(1500 - np.interp(hz, hzs, offs)))


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


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM13"
    prefix = sys.argv[2] if len(sys.argv) > 2 else "tune"
    sr = 32000

    plan = [(BASE * 2 ** (s / 12), ms) for s, ms in MELODY]
    for hz, _ in plan:
        us_for(hz)  # refuse a tune the servo cannot reach before it starts

    ser, audio = open_and_sync(port)

    def listen(ms):
        end = time.time() + ms / 1000
        while time.time() < end:
            n = ser.in_waiting
            if n:
                audio.extend(ser.read(n))
            else:
                time.sleep(0.002)

    listen(600)  # a little silence first
    marks = []
    prev = None
    for hz, ms in plan:
        if prev is not None and abs(hz - prev) < 1:
            # Repeated note: drop to stop briefly, or the two merge into one.
            ser.write(b"W 1500\n"); listen(60); ms -= 60
        marks.append((len(audio) // 2, hz, ms))
        ser.write(f"W {us_for(hz)}\n".encode())
        listen(ms)
        prev = hz
    ser.write(b"W 1500\n")
    listen(800)
    ser.close()

    if len(audio) % 2:
        audio = audio[:-1]
    x = np.frombuffer(bytes(audio), dtype="<i2").astype(np.float64)
    with wave.open(f"{prefix}.wav", "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
        w.writeframes(x.astype("<i2").tobytes())
    print(f"Saved {prefix}.wav ({len(x)/sr:.1f}s, peak {int(np.max(np.abs(x)))}/32767)")

    print()
    print(f"{'note':>5} {'aimed':>9} {'played':>9} {'off by':>10}")
    names = ["G", "G#", "A", "A#", "B", "C", "C#", "D"]
    for (start, hz, ms), (semi, _) in zip(marks, MELODY):
        # Measure the back half of each note, after the motor has got to speed.
        a = start + int(ms * 0.5 / 1000 * sr)
        b = start + int(ms / 1000 * sr)
        seg = x[a:b] - x[a:b].mean()
        n = 1 << 16
        spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg)), n))
        f = np.fft.rfftfreq(n, 1 / sr)
        band = (f >= 700) & (f <= 1600)
        got = f[band][np.argmax(spec[band])]
        cents = 1200 * np.log2(got / hz)
        print(f"{names[semi]:>5} {hz:8.0f}Hz {got:8.0f}Hz {cents:+9.0f}c")
    print()
    print("100 cents = one semitone. Within about 30 either way reads as in tune.")


if __name__ == "__main__":
    main()
