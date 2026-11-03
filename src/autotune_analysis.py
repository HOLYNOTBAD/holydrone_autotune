"""计算离线脉冲跟踪指标，按候选、方向和幅值汇总中位数及四分位距。
角速度采用 FRD rad/s，时间采用 PX4 秒；不可测指标返回 None，不写回参数。"""
import numpy as np


def integral(y, t):
    return float(np.trapz(y, t))


def pulse_metrics(t, actual, target, return_t, return_actual, hf_hz=30):
    """计算单个恒定目标脉冲的跟踪与回零指标。

    Args:
        t: 严格递增的 PX4 时间，s。
        actual: FRD 实测角速度，rad/s。
        target: 本段目标角速度，rad/s。
        return_t: 紧邻零指令窗口的时间，s；允许为空。
        return_actual: 零指令窗口的实测角速度，rad/s。
        hf_hz: 高频能量截止频率，Hz。

    Returns:
        指标字典；未观测到或不可测的项目为 None。"""
    t, actual = np.asarray(t), np.asarray(actual)
    if len(t) < 4 or np.any(np.diff(t) <= 0):
        raise ValueError('insufficient or non-monotonic pulse samples')
    error = target - actual
    # 以首个实测值到本段目标的阶跃定义 10–90% 上升时间，兼容正负反转。
    step = target - actual[0]
    progress = (actual-actual[0])/step if abs(step) > 1e-9 else np.zeros_like(actual)
    i10, i90 = np.flatnonzero(progress >= .1), np.flatnonzero(progress >= .9)
    rise = float(t[i90[0]]-t[i10[0]]) if len(i10) and len(i90) else None
    signed = np.sign(target)*actual
    dt = np.median(np.diff(t))
    regular = np.arange(t[0], t[-1], dt)
    signal = np.interp(regular, t, actual)
    # 去均值后的单边频谱求和得到截止频率以上能量；单位 (rad/s)²。
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
    # 进入幅值 5% 以内且保持到零指令窗口结束至少 0.1 s，才判为回零。
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
    """按候选、幅值与方向汇总重复脉冲，报告中位数、四分位距和有效计数。"""
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
