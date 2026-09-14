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

def read_audio(ser, total_ms, sr, prefix=b""):
    n_samples = int(sr * (total_ms / 1000.0)) + sr  # pad a second
    n_bytes = n_samples * 2
    data = bytearray(prefix)
    while len(data) < n_bytes:
        chunk = ser.read(min(8192, n_bytes - len(data)))
        if not chunk:
            break
        data += chunk
    if len(data) % 2:  # keep whole samples only
        data = data[:-1]
    return np.frombuffer(bytes(data), dtype="<i2")

def save_wav(path, samples, sr):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
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

def avg_spectrum(seg, sr, n=4096):
    """Averaged power spectrum. Returns (freqs, power) or (None, None)."""
    if len(seg) < n:
        n = 1 << (len(seg).bit_length() - 1)
        if n < 256:
            return None, None
    win = np.hanning(n)
    acc = np.zeros(n // 2 + 1)
    k = 0
    for i in range(0, len(seg) - n + 1, n // 2):
        acc += np.abs(np.fft.rfft(seg[i:i + n] * win)) ** 2
        k += 1
    if k == 0:
        return None, None
    return np.fft.rfftfreq(n, 1 / sr), acc / k

def spectral_stats(segment, sr, lo=500.0):
    """Describe the segment's spectrum above `lo`.

    peak_hz / centroid_hz say where the energy is; flatness says whether it is
    a tone or a noise band (near 1.0 = noise-like, below ~0.1 = strongly tonal).
    This is the honest measure for servo noise: a pitch detector will always
    return some number, but flatness says whether that number means anything.
    """
    seg = segment.astype(np.float64)
    seg -= seg.mean()
    f, p = avg_spectrum(seg, sr)
    if f is None:
        return None, None, None
    m = (f >= lo) & (f <= sr * 0.47)
    if not m.any() or p[m].sum() <= 0:
        return None, None, None
    fb, pb = f[m], p[m]
    peak = float(fb[np.argmax(pb)])
    centroid = float((fb * pb).sum() / pb.sum())
    pos = pb[pb > 0]
    flatness = float(np.exp(np.mean(np.log(pos))) / np.mean(pos)) if len(pos) else None
    return peak, centroid, flatness

def detect_pitch(segment, sr, fmin=60, fmax=None, threshold=0.55, min_conf=0.30):
    """Autocorrelation pitch detector.

    Returns (freq, confidence) or (None, 0.0) when there is no usable period.

    A plain argmax over the lag search window always returns something. On
    broadband noise it returns the lag right next to zero, because that peak is
    just the signal's high-frequency rolloff, and the peak/zero-lag ratio stays
    high enough to look like confidence. The first real recording came back with
    all 108 events reporting one of three frequencies, every one of them sitting
    on min_lag. So a peak resting on either edge of the search window is
    reported as no-pitch rather than as a number.
    """
    if fmax is None:
        fmax = sr * 0.45
    seg = segment.astype(np.float64)
    seg -= seg.mean()
    if np.max(np.abs(seg)) < 50:  # near-silence, INMP441 16-bit-ish counts
        return None, 0.0
    windowed = seg * np.hanning(len(seg))
    # Autocorrelation via FFT. np.correlate does this directly, which is O(n^2)
    # and took minutes once segments reached 60k samples at 32 kHz. Same result.
    n = 1 << (2 * len(windowed) - 1).bit_length()
    spec = np.fft.rfft(windowed, n)
    corr = np.fft.irfft(spec * np.conj(spec), n)[:len(windowed)]
    min_lag = max(2, int(sr / fmax))
    max_lag = int(sr / fmin)
    if max_lag >= len(corr):
        max_lag = len(corr) - 1
    if min_lag >= max_lag or corr[0] <= 0:
        return None, 0.0

    r = corr[:max_lag + 1] / corr[0]
    search = r[min_lag:max_lag]
    if len(search) < 5:
        return None, 0.0

    # Take the FIRST strong peak, not the largest. A periodic signal correlates
    # just as well at twice its period, so the global argmax lands an octave
    # low about as often as not: a clean 250 Hz tone came back as 125 Hz.
    interior = np.r_[False, (search[1:-1] >= search[:-2]) & (search[1:-1] > search[2:]), False]
    strong = interior & (search >= threshold * search.max())
    # Peaks hard against either edge are the search window running out, not a
    # period: broadband noise otherwise reports the lag beside zero at a
    # confidence high enough to pass for a real reading. Skip those and keep
    # looking, rather than giving up on the whole segment because of one.
    idx = [i for i in np.flatnonzero(strong) if 1 < i < len(search) - 2]
    if not idx:
        return None, 0.0
    peak_off = int(idx[0])
    conf = float(search[peak_off])
    if conf < min_conf:
        return None, 0.0
    return sr / (min_lag + peak_off), conf

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    port, prefix = sys.argv[1], sys.argv[2]

    # A read started while the port is still re-enumerating (right after a
    # flash) can lose a byte mid-plan, which surfaces as a JSON error rather
    # than as the dropped byte it is. Retry rather than dying.
    #
    # Each attempt reopens the port. Pulsing DTR/RTS on a connection that is
    # already mid-stream does not reliably reset this board, so retries would
    # otherwise just read more of the audio already in flight and time out
    # waiting for a plan that was never going to be printed again.
    ser = None
    for attempt in range(1, 4):
        if ser is not None:
            ser.close()
            time.sleep(1.0)
        ser = serial.Serial(port, 921600, timeout=5)
        print(f"Resetting board (attempt {attempt})...", flush=True)
        ser.reset_input_buffer()
        reset_board(ser)
        print("Waiting for test plan...", flush=True)
        try:
            plan, leftover = read_plan(ser)
            break
        except (json.JSONDecodeError, RuntimeError) as exc:
            first = str(exc).split(". Last bytes")[0]
            print(f"  plan unreadable ({first}); retrying", flush=True)
            if attempt == 3:
                raise
    events = plan["events"]
    total_ms = plan["total_ms"]
    sr = int(plan.get("sample_rate", SAMPLE_RATE))
    print(f"Got plan: {len(events)} events, {total_ms} ms "
          f"({total_ms/60000:.1f} min) at {sr} Hz. Recording...", flush=True)

    samples = read_audio(ser, total_ms, sr, prefix=leftover)

    # Every slice is an offset in samples, so a short stream silently shifts
    # every event after the gap. Say so rather than analysing shifted audio.
    got_ms = len(samples) / sr * 1000.0
    if got_ms < total_ms:
        print(f"WARNING: expected {total_ms} ms of audio, got {got_ms:.0f} ms "
              f"({total_ms - got_ms:.0f} ms short). Samples were dropped, so "
              f"event alignment past that point is not trustworthy.", flush=True)

    # Clipping fabricates broadband harmonics, which is exactly the thing the
    # flatness measure is trying to read. A clipped recording has to say so.
    clipped = int(np.sum(np.abs(samples) >= 32000))
    if clipped:
        pct = 100.0 * clipped / max(len(samples), 1)
        print(f"WARNING: {clipped} samples ({pct:.2f}%) at full scale. The mic is "
              f"overloading, so tonality and spectrum are not trustworthy. "
              f"Raise MIC_SHIFT in the firmware or move the mic back.", flush=True)

    wav_path = f"{prefix}.wav"
    save_wav(wav_path, samples, sr)
    print(f"Saved {wav_path} ({len(samples)/sr:.1f}s, peak {int(np.max(np.abs(samples)))}/32767)",
          flush=True)

    # Keep the plan next to the audio so the analysis can be re-run later
    # without re-recording, and without trusting a reconstruction of it.
    with open(f"{prefix}.plan.json", "w") as f:
        json.dump(plan, f)

    rows = []
    for ev in events:
        start = int(ev["start_ms"] / 1000.0 * sr)
        dur = int(ev["dur_ms"] / 1000.0 * sr)
        # Skip the mechanical onset of a long move. A sketch whose events ARE
        # the onset (single hits) says so in its plan with skip_ms 0: a fixed
        # 100 ms skip measured only the servo holding still after 2-12 degree
        # moves that were over by 40 ms.
        skip = int(plan.get("skip_ms", 100) / 1000.0 * sr)
        seg = samples[start + skip : start + dur]
        if len(seg) < 256:
            continue
        freq, conf = detect_pitch(seg, sr)
        peak, centroid, flat = spectral_stats(seg, sr)
        note_info = nearest_note(freq) if freq else None
        rows.append({
            "i": ev["i"], "type": ev["type"], "p1": ev["p1"], "p2": ev["p2"],
            "freq_hz": round(freq, 1) if freq else None,
            "confidence": round(conf, 3),
            "note": note_info[0] if note_info else None,
            "cents_off": round(note_info[2], 1) if note_info else None,
            "peak_hz": round(peak, 1) if peak else None,
            "centroid_hz": round(centroid, 1) if centroid else None,
            "flatness": round(flat, 3) if flat else None,
        })

    csv_path = f"{prefix}.csv"
    cols = ["i", "type", "p1", "p2", "freq_hz", "confidence", "note",
            "cents_off", "peak_hz", "centroid_hz", "flatness"]
    with open(csv_path, "w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r[c]) for c in cols) + "\n")
    print(f"Saved {csv_path} ({len(rows)} events)", flush=True)

    pitched = sum(1 for r in rows if r["freq_hz"] is not None)
    flats = [r["flatness"] for r in rows if r["flatness"] is not None]
    print(f"  {pitched} of {len(rows)} events had a usable period; "
          f"median flatness {np.median(flats):.3f} "
          f"(1.0 = noise-like, below 0.1 = strongly tonal)", flush=True)

if __name__ == "__main__":
    main()
