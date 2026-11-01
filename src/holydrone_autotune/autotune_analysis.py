"""Offline metrics only. All angular quantities are FRD rad/s; time is PX4 seconds."""
import numpy as np


def integral(y, t):
    return float(np.trapz(y, t))


def pulse_metrics(t, actual, target, return_t, return_actual, hf_hz=30):
    t, actual = np.asarray(t), np.asarray(actual)
    if len(t) < 4 or np.any(np.diff(t) <= 0):
        raise ValueError('insufficient or non-monotonic pulse samples')
    error = target - actual
    # Step-relative 10–90% rise handles the +A -> -A reversal too.
    step = target - actual[0]
    progress = (actual-actual[0])/step if abs(step) > 1e-9 else np.zeros_like(actual)
    i10, i90 = np.flatnonzero(progress >= .1), np.flatnonzero(progress >= .9)
    rise = float(t[i90[0]]-t[i10[0]]) if len(i10) and len(i90) else None
    signed = np.sign(target)*actual
    dt = np.median(np.diff(t))
    regular = np.arange(t[0], t[-1], dt)
    signal = np.interp(regular, t, actual)
    # Integrated one-sided periodogram above hf_hz; units (rad/s)^2.
    signal = signal - np.mean(signal)
    spectrum = np.abs(np.fft.rfft(signal))**2 / len(signal)**2
    if len(signal) % 2 == 0:
        spectrum[1:-1] *= 2
    else:
        spectrum[1:] *= 2
    freq = np.fft.rfftfreq(len(signal), dt)
    hf = float(np.sum(spectrum[freq >= hf_hz])) if hf_hz < .5/dt else None
    rt, ry = np.asarray(return_t), np.asarray(return_actual)
    return_time = None
    # Within 5% of amplitude for the remainder of this zero window, at least 0.1 s.
    if len(rt):
        outside = np.flatnonzero(np.abs(ry) > .05*abs(target))
        start = outside[-1]+1 if len(outside) else 0
        if start < len(rt) and rt[-1]-rt[start] >= .1:
            return_time = float(rt[start]-rt[0])
    return dict(rise_time_s=rise,
                overshoot_percent=float(max(0, np.max(signed)-abs(target))/abs(target)*100),
                iae=integral(np.abs(error), t),
                itae=integral((t-t[0])*np.abs(error), t),
                tracking_rmse=float(np.sqrt(np.mean(error**2))),
                return_time_s=return_time,
                return_rms=float(np.sqrt(np.mean(ry**2))) if len(ry) else None,
                high_frequency_energy=hf)


def summarize(rows):
    """Median/IQR across repeated pulses, preserving direction and amplitude."""
    groups = {}
    for row in rows:
        key = (row['candidate_id'], row['amplitude_deg_s'], row['direction'])
        groups.setdefault(key, []).append(row)
    result = []
    excluded = {'trial_id', 'repetition', 'phase', 'amplitude_deg_s', 'candidate_id', 'direction'}
    for key, values in groups.items():
        entry = dict(candidate_id=key[0], amplitude_deg_s=key[1], direction=key[2], pulse_count=len(values))
        for metric in values[0]:
            if metric in excluded:
                continue
            numbers = [v[metric] for v in values if isinstance(v[metric], (float, int)) and np.isfinite(v[metric])]
            if numbers:
                entry[metric] = dict(median=float(np.median(numbers)),
                                     iqr=float(np.percentile(numbers, 75)-np.percentile(numbers, 25)),
                                     count=len(numbers))
        result.append(entry)
    return result
