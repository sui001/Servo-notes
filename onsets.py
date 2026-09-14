"""Burst-onset timing. Validated against synthetic ticks before use."""
import numpy as np

def onset_times(seg, sr, band=(3000, 15000), smooth_ms=1.5, min_gap_ms=6.0):
    """Return onset times (seconds) of broadband bursts in seg.

    Works on the band-limited envelope, takes the positive part of its
    derivative (spectral-flux style), and picks peaks above an adaptive
    threshold with a refractory gap so one burst cannot register twice.
    """
    x = np.asarray(seg, dtype=np.float64)
    x = x - x.mean()
    if len(x) < 512:
        return np.array([])
    # band-limit via FFT so low-frequency room noise cannot drive onsets
    n = len(x)
    f = np.fft.rfftfreq(n, 1 / sr)
    X = np.fft.rfft(x)
    X[(f < band[0]) | (f > band[1])] = 0
    xb = np.fft.irfft(X, n)

    env = np.abs(xb)
    k = max(1, int(smooth_ms * 1e-3 * sr))
    env = np.convolve(env, np.ones(k) / k, "same")

    d = np.diff(env, prepend=env[0])
    d[d < 0] = 0.0
    # smooth the flux too, or every sample of noise is its own local maximum
    kd = max(1, int(0.5e-3 * sr))
    d = np.convolve(d, np.ones(kd) / kd, "same")
    if d.max() <= 0:
        return np.array([])

    # Absolute (MAD) and relative (fraction of the largest flux) thresholds.
    # With MAD alone the bar sits inside the noise, every wiggle qualifies, and
    # the refractory gap ends up setting the reported rate: noise with no ticks
    # at all came back as a confident 164.9 Hz, which is exactly 1/min_gap.
    mad = np.median(np.abs(d - np.median(d))) * 1.4826
    thr = max(np.median(d) + 4.0 * mad, 0.25 * d.max())
    refractory = int(min_gap_ms * 1e-3 * sr)

    onsets = []
    i = 1
    last = -10 ** 9
    while i < len(d) - 1:
        if d[i] > thr and d[i] >= d[i - 1] and d[i] > d[i + 1] and (i - last) > refractory:
            onsets.append(i / sr)
            last = i
        i += 1
    return np.array(onsets)

def rate_from_onsets(on):
    """Median inter-onset rate in Hz, or None."""
    if len(on) < 3:
        return None, None
    ioi = np.diff(on)
    ioi = ioi[ioi > 0]
    if len(ioi) < 2:
        return None, None
    med = float(np.median(ioi))
    # consistency: how tight the intervals are (low = regular)
    spread = float(np.median(np.abs(ioi - med)) / med) if med > 0 else None
    return (1.0 / med if med > 0 else None), spread

if __name__ == "__main__":
    rng = np.random.default_rng(4)
    sr = 32000
    print("%-38s %12s %12s %9s" % ("synthetic input", "true rate", "measured", "spread"))
    ok = True
    for rate in (10, 16.7, 25, 50, 100):
        dur = 2.0
        t = np.arange(int(sr * dur)) / sr
        sig = 300 * rng.standard_normal(len(t))
        period = int(sr / rate)
        for start in range(0, len(t) - 400, period):
            burst = rng.standard_normal(300) * 4000 * np.exp(-np.linspace(0, 6, 300))
            # make bursts broadband but centred high, like the servo tick
            sig[start:start + 300] += burst
        on = onset_times(sig, sr)
        r, sp = rate_from_onsets(on)
        good = r is not None and abs(r - rate) / rate < 0.10
        ok &= good
        print("%-38s %12.1f %12s %9s  %s" % (
            "%.1f Hz tick train" % rate, rate,
            "%.1f Hz" % r if r else "none",
            "%.2f" % sp if sp is not None else "-",
            "OK" if good else "WRONG"))
    # negative control: no ticks at all
    noise = 300 * rng.standard_normal(sr * 2)
    on = onset_times(noise, sr)
    r, sp = rate_from_onsets(on)
    print("%-38s %12s %12s %9s  %s" % (
        "noise, no ticks", "none", "%.1f Hz" % r if r else "none",
        "%.2f" % sp if sp is not None else "-",
        "(spread should be high / irregular)"))
    print()
    print("ALL CORRECT" if ok else "STILL BROKEN")
