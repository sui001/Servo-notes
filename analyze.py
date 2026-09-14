#!/usr/bin/env python3
"""
Reads the JSON plan + raw PCM stream from servo_mic_test.ino over serial,
saves a WAV, and for each planned event runs pitch detection and reports
the nearest equal-tempered note and how far off it is.

Usage:
    python3 analyze.py /dev/ttyACM0 out_prefix
    (Windows: python3 analyze.py COM7 out_prefix)

Requires: pyserial, numpy. Nothing else (WAV writing uses stdlib `wave`).

UNTESTED end-to-end (no hardware here to run it against) — the JSON
parsing and WAV writing are solid stdlib operations; the pitch detector
is a plain autocorrelation and may need tuning (see NOTE below) once you
see what real servo-noise recordings look like.
"""
import sys
import json
import time
import wave
import numpy as np
import serial

SAMPLE_RATE = 16000
DELIM = b"===AUDIO_START==="

def reset_board(ser):
    """Pulse the board into a fresh boot.

    The plan is printed once, at boot. Without a reset here the script waits
    for a delimiter that already went past while it was connecting, and hangs
    with no output at all.
    """
    ser.setDTR(False)
    ser.setRTS(True)
    time.sleep(0.1)
    ser.setRTS(False)
    time.sleep(0.2)

def read_plan(ser, timeout_s=20):
    buf = b""
    deadline = time.time() + timeout_s
    while DELIM not in buf:
        if time.time() > deadline:
            raise RuntimeError(
                f"no test plan in {timeout_s}s. Last bytes seen: {buf[-200:]!r}"
            )
        buf += ser.read(512)
    header, _, leftover = buf.partition(DELIM)
    # The ROM prints its own boot banner before our JSON, so start at the JSON.
    start = header.find(b'{"sample_rate"')
    if start < 0:
        raise RuntimeError(f"no plan JSON in header: {header[-300:]!r}")
    plan = json.loads(header[start:].decode("utf-8", errors="replace"))
    # Audio can arrive in the same read as the delimiter. Keeping it matters:
    # every slice is an offset from the first audio sample.
    return plan, leftover.lstrip(b"\r\n")

def read_audio(ser, total_ms, prefix=b""):
    n_samples = int(SAMPLE_RATE * (total_ms / 1000.0)) + SAMPLE_RATE  # pad a second
    n_bytes = n_samples * 2
    data = bytearray(prefix)
    while len(data) < n_bytes:
        chunk = ser.read(min(4096, n_bytes - len(data)))
        if not chunk:
            break
        data += chunk
    if len(data) % 2:  # keep whole samples only
        data = data[:-1]
    return np.frombuffer(bytes(data), dtype="<i2")

def save_wav(path, samples):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(samples.tobytes())

NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]

def nearest_note(freq_hz):
    if freq_hz <= 0:
        return None
    # MIDI note number relative to A4=440 (MIDI 69)
    midi = 69 + 12 * np.log2(freq_hz / 440.0)
    nearest_midi = int(round(midi))
    cents_off = (midi - nearest_midi) * 100
    name = NOTE_NAMES[nearest_midi % 12]
    octave = nearest_midi // 12 - 1
    target_hz = 440.0 * 2 ** ((nearest_midi - 69) / 12.0)
    return f"{name}{octave}", target_hz, cents_off

def detect_pitch(segment, sr=SAMPLE_RATE, fmin=60, fmax=4000):
    """Plain autocorrelation pitch detector.
    NOTE: servo whine may be inharmonic/noisy rather than a clean tone —
    if results look garbage, try band-limiting (a bandpass around where
    you can hear the zipt) before autocorrelating, or fall back to just
    taking the FFT peak instead.
    """
    seg = segment.astype(np.float64)
    seg -= seg.mean()
    if np.max(np.abs(seg)) < 50:  # near-silence, INMP441 16-bit-ish counts
        return None, 0.0
    windowed = seg * np.hanning(len(seg))
    corr = np.correlate(windowed, windowed, mode="full")
    corr = corr[len(corr) // 2:]
    min_lag = int(sr / fmax)
    max_lag = int(sr / fmin)
    if max_lag >= len(corr):
        max_lag = len(corr) - 1
    if min_lag >= max_lag:
        return None, 0.0
    search = corr[min_lag:max_lag]
    peak_lag = min_lag + int(np.argmax(search))
    if corr[0] == 0:
        return None, 0.0
    confidence = search.max() / corr[0]
    freq = sr / peak_lag
    return freq, confidence

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    port, prefix = sys.argv[1], sys.argv[2]

    ser = serial.Serial(port, 921600, timeout=5)
    print("Resetting board...", flush=True)
    reset_board(ser)
    print("Waiting for test plan...", flush=True)
    plan, leftover = read_plan(ser)
    events = plan["events"]
    total_ms = plan["total_ms"]
    print(f"Got plan: {len(events)} events, {total_ms} ms "
          f"({total_ms/60000:.1f} min). Recording...", flush=True)

    samples = read_audio(ser, total_ms, prefix=leftover)
    wav_path = f"{prefix}.wav"
    save_wav(wav_path, samples)
    print(f"Saved {wav_path} ({len(samples)/SAMPLE_RATE:.1f}s)", flush=True)

    rows = []
    for ev in events:
        start = int(ev["start_ms"] / 1000.0 * SAMPLE_RATE)
        dur = int(ev["dur_ms"] / 1000.0 * SAMPLE_RATE)
        # Skip the first ~100ms (mechanical onset transient), use the rest.
        skip = int(0.1 * SAMPLE_RATE)
        seg = samples[start + skip : start + dur]
        if len(seg) < 256:
            continue
        freq, conf = detect_pitch(seg)
        note_info = nearest_note(freq) if freq else None
        rows.append({
            "i": ev["i"], "type": ev["type"], "p1": ev["p1"], "p2": ev["p2"],
            "freq_hz": round(freq, 1) if freq else None,
            "confidence": round(conf, 3),
            "note": note_info[0] if note_info else None,
            "cents_off": round(note_info[2], 1) if note_info else None,
        })

    csv_path = f"{prefix}.csv"
    with open(csv_path, "w") as f:
        f.write("i,type,p1,p2,freq_hz,confidence,note,cents_off\n")
        for r in rows:
            f.write(f"{r['i']},{r['type']},{r['p1']},{r['p2']},{r['freq_hz']},"
                    f"{r['confidence']},{r['note']},{r['cents_off']}\n")
    print(f"Saved {csv_path} ({len(rows)} events)", flush=True)

if __name__ == "__main__":
    main()
