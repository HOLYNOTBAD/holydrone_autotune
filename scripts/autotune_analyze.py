#!/usr/bin/env python3
"""分析事件和 PX4 ULog；按脉冲结束事件截断，回中阶段不作为零输入窗口。"""
import argparse
import csv
import json
import sys
import tempfile
from pathlib import Path
import numpy as np
import yaml
# 源码和安装空间均可直接运行，无需 ROS 环境。
PACKAGE = Path(__file__).resolve().parents[1]
if not (PACKAGE / 'package.xml').is_file():
    PACKAGE = Path(__file__).resolve().parents[2] / 'share/holydrone_autotune'

CORE_METRICS = ('tracking_rmse', 'overshoot_percent', 'rise_time_s')


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


def write_summary_tables(pulses, path):
    """合并同候选/幅值的正负脉冲，按幅值升序输出平均指标 CSV。"""
    fields = ['candidate_id', 'amplitude_deg_s', 'tracking_rmse_mean_rad_s',
              'overshoot_percent_mean', 'rise_time_mean_ms']
    groups = {}
    for pulse in pulses:
        groups.setdefault((pulse['candidate_id'], pulse['amplitude_deg_s']), []).append(pulse)
    rows = []
    for (candidate, amplitude), values in sorted(groups.items(), key=lambda item: item[0][1]):
        means = []
        for metric in CORE_METRICS:
            valid = [v[metric] for v in values
                     if v.get(metric) is not None and np.isfinite(v[metric])]
            mean = float(np.mean(valid)) if valid else None
            means.append(mean*1000 if metric == 'rise_time_s' and mean is not None else mean)
        rows.append([candidate, amplitude, *means])
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True, help='directory containing events.yaml and timesync.csv')
    p.add_argument('--ulog', type=Path, help='ULog path; default: unique *.ulg under <session>_px4')
    p.add_argument('--output', type=Path, help='new output directory; default: <session>_result')
    p.add_argument('--ros-minus-px4-s', type=float, help='explicit measured clock offset; overrides TIMESYNC')
    p.add_argument('--hf-hz', type=float, default=30)
    args = p.parse_args()
    if not np.isfinite(args.hf_hz) or args.hf_hz <= 0:
        p.error('--hf-hz must be positive')
    if args.ros_minus_px4_s is not None and not np.isfinite(args.ros_minus_px4_s):
        p.error('--ros-minus-px4-s must be finite')
    session = Path(args.session).expanduser()
    if not session.is_dir() and len(session.parts) == 1:
        session = PACKAGE / 'data' / session
    session = session.resolve()
    if not (session / 'events.yaml').is_file():
        p.error('session must contain events.yaml: ' + str(session))
    destination = (args.output.expanduser() if args.output else session.with_name(session.name+'_result')).resolve()
    if destination.exists():
        p.error('output already exists; use a new --output directory: ' + str(destination))
    if args.ulog is None:
        files = sorted(session.with_name(session.name+'_px4').rglob('*.ulg'))
        if len(files) != 1:
            p.error('expected one ULog under <session>_px4; found %d; specify --ulog explicitly%s' %
                    (len(files), ''.join('\n  '+str(path) for path in files)))
        args.ulog = files[0]
    args.ulog = args.ulog.expanduser().resolve()
    if not args.ulog.is_file() or args.ulog.suffix.lower() != '.ulg':
        p.error('--ulog must be an existing .ulg file')
    print('Session:', session, flush=True)
    print('ULog:', args.ulog, flush=True)
    print('Output:', destination, flush=True)
    if args.ros_minus_px4_s is not None:
        print('Explicit ROS-minus-PX4 offset: %g s; FCU pulse plateaus will be checked.' % args.ros_minus_px4_s, flush=True)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # 全部分析成功后才发布结果目录，失败或中断时清理临时输出。
    with tempfile.TemporaryDirectory(prefix='.'+destination.name+'-', dir=destination.parent) as temporary:
        analyze(args, session, Path(temporary))
        if destination.exists():
            raise FileExistsError('output was created during analysis: '+str(destination))
        Path(temporary).rename(destination)
    print('Results saved:', destination)


def resolve_clock(events, sync, override, ulog_info, setpoint_t, setpoint, stream_times):
    """优先用 TIMESYNC；只对明确的 SITL 日志验证共享 boot 时钟回退。"""
    info = dict(source='explicit' if override is not None else 'timesync',
                duplicate_samples=0, timesync_error=None)
    stamps = [event['timestamp'] for event in events if event and 'timestamp' in event]
    if np.any(np.diff(stamps) < 0):
        raise ValueError('ROS event clock moved backwards; split the session at the reset')
    if override is not None:
        info['ros_minus_px4_s'] = override
        return lambda event: event['timestamp']-override, info
    sync = np.asarray(sync, dtype=float)
    if sync.size and sync.ndim == 2 and sync.shape[1] == 3:
        delta = np.diff(sync[:, 0])
        if np.any(delta < 0):
            raise ValueError('ROS clock moved backwards in TIMESYNC; split the session at the reset')
        info['duplicate_samples'] = int(np.count_nonzero(delta == 0))
        # 同一个 ROS 时刻的重复回调保留最后一条，不把它误判成时钟回退。
        sync = sync[np.r_[delta != 0, True]]

    trials = {}
    for event in events:
        if event and event.get('trial_id', 0):
            trials.setdefault(event['trial_id'], []).append(event)
    windows, spans = [], []
    for trial in trials.values():
        starts = [e for e in trial if e['event_type'] == 'TRIAL_START']
        ends = [e for e in trial if e['event_type'] in ('TRIAL_FINISHED', 'TRIAL_ABORTED')]
        if not starts or not ends or ends[0]['event_type'] != 'TRIAL_FINISHED':
            continue
        spans.append((starts[0], ends[0]))
        boundaries = [e for e in trial if e['event_type'] in
                      ('PULSE_POSITIVE', 'PULSE_NEGATIVE', 'PULSE_ZERO', 'PULSE_FINISHED', 'RECOVERY_START')]
        boundaries.append(ends[0])
        for index, event in enumerate(boundaries[:-1]):
            if event['event_type'] in ('PULSE_POSITIVE', 'PULSE_NEGATIVE'):
                windows.append((event, boundaries[index+1]))

    def validate(convert, check_edges=False):
        if not windows:
            raise ValueError('no completed pulse trials for clock validation')
        for start, end in spans:
            a, b = convert(start), convert(end)
            if any(times[0] > a or times[-1] < b for times in stream_times):
                raise ValueError('mapped trial falls outside ULog coverage')
        errors, delays = [], []
        for start, end in windows:
            a, b = convert(start), convert(end)
            if b <= a:
                raise ValueError('mapped pulse has nonpositive duration')
            sign = 1 if start['event_type'] == 'PULSE_POSITIVE' else -1
            target = sign*np.deg2rad(start['amplitude_deg_s'])
            signal = np.asarray(setpoint[start['axis']])
            middle = (setpoint_t >= a+.03) & (setpoint_t < b-.03)
            mismatch = float(np.median(np.abs(signal[middle]-target))) if np.any(middle) else float('inf')
            if mismatch > max(.05, .15*abs(target)):
                raise ValueError('event/ULog setpoint mismatch')
            errors.append(mismatch)
            if check_edges:
                matching = np.flatnonzero((setpoint_t >= a-.06) & (setpoint_t <= a+.06) &
                                          (np.abs(signal-target) < .01))
                if not len(matching):
                    raise ValueError('no matching FCU setpoint near a pulse start')
                delays.append(float(setpoint_t[matching[0]]-a))
        return dict(validated_pulses=len(windows), max_median_setpoint_error_rad_s=max(errors),
                    observed_edge_delay_range_s=[min(delays), max(delays)] if delays else None)

    try:
        if sync.ndim != 2 or len(sync) < 10 or not np.isfinite(sync).all():
            raise ValueError('missing or invalid TIMESYNC samples')
        def synced(event):
            stamp = event['timestamp']
            if stamp < sync[0, 0] or stamp > sync[-1, 0] or np.min(np.abs(sync[:, 0]-stamp)) > 2:
                raise ValueError('TIMESYNC does not cover trial events')
            return stamp-float(np.interp(stamp, sync[:, 0], sync[:, 1]))
        info.update(validate(synced))
        return synced, info
    except ValueError as error:
        info['timesync_error'] = str(error)

    if ulog_info.get('ver_hw') == 'PX4_SITL':
        if len(windows) < 4 or {e['event_type'] for e, _ in windows} != {'PULSE_POSITIVE', 'PULSE_NEGATIVE'}:
            raise ValueError('insufficient positive/negative pulses to validate SITL shared clock; supply a measured --ros-minus-px4-s')
        shared = lambda event: event['timestamp']
        try:
            info.update(validate(shared, check_edges=True))
        except ValueError as error:
            raise ValueError('TIMESYNC is unusable and SITL shared clock validation failed: '+str(error)+
                             '; check selected ULog or supply a measured --ros-minus-px4-s') from error
        info.update(source='sitl_shared_clock_validated', ros_minus_px4_s=0.0,
                    note='Zero offset accepted only for PX4_SITL after every completed pulse matched. Edge delay includes clock, transport and sampling uncertainty.')
        return shared, info
    raise ValueError('TIMESYNC is unusable: '+info['timesync_error']+
                     '; supply a measured --ros-minus-px4-s. Automatic zero offset is restricted to validated PX4_SITL logs.')


def analyze(args, session, output):
    from pyulog import ULog
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    events = list(yaml.safe_load_all((session / 'events.yaml').read_text()))
    sync = []
    if args.ros_minus_px4_s is None and (session / 'timesync.csv').is_file():
        with (session / 'timesync.csv').open(newline='') as stream:
            sync = [(float(row['ros_s']), float(row['ros_minus_px4_s']), float(row['round_trip_time_ms']))
                    for row in csv.DictReader(stream)]
    ulog = ULog(str(args.ulog))
    def dataset(name):
        data = ulog.get_dataset(name).data
        t = np.asarray(data['timestamp'], dtype=float)*1e-6
        # 状态事件可在同一PX4时刻多次更新；连续控制信号仍要求严格递增。
        limit = np.diff(t) < 0 if name in ('vehicle_status', 'battery_status') else np.diff(t) <= 0
        if len(t) < 2 or np.any(limit):
            raise ValueError('invalid timestamps: '+name)
        return t, data
    ta, actual = dataset('vehicle_angular_velocity')
    ts, setpoint = dataset('vehicle_rates_setpoint')
    tt, torque = dataset('vehicle_torque_setpoint')
    tc, allocator = dataset('control_allocator_status')
    for topic in ['vehicle_attitude', 'vehicle_local_position', 'battery_status', 'vehicle_status']:
        dataset(topic)  # Explicitly report missing required ULog streams.
    trials, rows, skipped, alignment = {}, [], [], []
    plot_groups = {}
    for e in events:
        if e and e.get('trial_id', 0):
            trials.setdefault(e['trial_id'], []).append(e)
    px4_time, clock_info = resolve_clock(events, sync, args.ros_minus_px4_s,
                                         ulog.msg_info_dict, ts, setpoint, [ta, ts, tt, tc])
    clock_config = yaml.safe_load((session/'clock.yaml').read_text()) if (session/'clock.yaml').is_file() else {}
    clock_info['recorded_configuration'] = clock_config
    print('Clock source:', clock_info['source'], flush=True)
    if clock_info['source'] == 'sitl_shared_clock_validated':
        if not sync and clock_config.get('use_sim_time') is True and clock_config.get('mavros_timesync_mode') == 'PASSTHROUGH':
            print('PASSTHROUGH: no TIMESYNC samples expected; shared clock verified against all %d pulses.' % clock_info['validated_pulses'], flush=True)
        else:
            print('TIMESYNC unavailable (%s); SITL shared clock independently verified (%d pulses).' %
                  (clock_info['timesync_error'], clock_info['validated_pulses']), flush=True)
    for trial_id, ev in trials.items():
        starts = [e for e in ev if e['event_type'] == 'TRIAL_START']
        ends = [e for e in ev if e['event_type'] in ('TRIAL_FINISHED', 'TRIAL_ABORTED')]
        if not starts or not ends or ends[0]['event_type'] != 'TRIAL_FINISHED':
            skipped.append(dict(trial_id=trial_id, reason='aborted/incomplete')); continue
        start, end = px4_time(starts[0]), px4_time(ends[0])
        axis = starts[0]['axis']; idx = ['roll', 'pitch', 'yaw'].index(axis)
        for times in [ta, ts, tt, tc]:
            if times[0] > start or times[-1] < end:
                raise ValueError('ULog does not cover trial. Check TIMESYNC offsets or supply a measured --ros-minus-px4-s; zero is not assumed.')
        pulses = [e for e in ev if e['event_type'] in ('PULSE_POSITIVE', 'PULSE_NEGATIVE', 'PULSE_ZERO', 'PULSE_FINISHED', 'RECOVERY_START')]
        boundaries = [(px4_time(e), e) for e in pulses] + [(end, ends[0])]
        av = np.asarray(actual['xyz[%d]' % idx]); sp = np.asarray(setpoint[axis])
        segments = []
        for j, (begin, e) in enumerate(boundaries[:-1]):
            if e['event_type'] not in ('PULSE_POSITIVE', 'PULSE_NEGATIVE'):
                continue
            finish, following = boundaries[j+1]
            if finish <= begin:
                raise ValueError('nonpositive pulse duration; check event ordering')
            mask = (ta >= begin) & (ta < finish)
            if np.count_nonzero(mask) < 4:
                raise ValueError('insufficient rate samples in pulse')
            sign = 1 if e['event_type'] == 'PULSE_POSITIVE' else -1
            target = sign*np.deg2rad(e['amplitude_deg_s'])
            # Check clock/sign/axis against the logged FCU setpoint, not just log overlap.
            sm = (ts >= begin+.03) & (ts < finish-.03)
            mismatch = float(np.median(np.abs(sp[sm]-target))) if np.any(sm) else float('inf')
            alignment.append(mismatch)
            if mismatch > max(.05, .15*abs(target)):
                raise ValueError('event/ULog setpoint mismatch: check clock offset, axis, or competing publisher')
            rm = np.zeros(len(ta), dtype=bool)
            if following['event_type'] == 'PULSE_ZERO' and j+2 < len(boundaries):
                rm = (ta >= finish) & (ta < boundaries[j+2][0])
            sm_plot = (ts >= begin) & (ts < finish)
            segments.append(dict(key=(e['repetition'], e['phase']), sign=sign,
                                 duration=finish-begin, actual_t=ta[mask]-begin, actual=av[mask],
                                 setpoint_t=ts[sm_plot]-begin, setpoint=sp[sm_plot]))
            metrics = pulse_metrics(ta[mask], av[mask], target, ta[rm], av[rm], args.hf_hz)
            metrics = {name: metrics[name] for name in CORE_METRICS}
            rows.append(dict(candidate_id=e['candidate_id'], trial_id=trial_id,
                             amplitude_deg_s=e['amplitude_deg_s'], repetition=e['repetition'],
                             phase=e['phase'], direction='positive' if sign > 0 else 'negative', **metrics))
        am, sm = [(t >= start) & (t <= end) for t in [ta, ts]]
        key = (axis, starts[0]['amplitude_deg_s'])
        plot_groups.setdefault(key, []).append(dict(
            trial_id=trial_id, candidate_id=starts[0]['candidate_id'],
            parameters=starts[0].get('readback_parameters') or {},
            segments=segments, actual_t=ta[am]-start, actual=av[am],
            setpoint_t=ts[sm]-start, setpoint=sp[sm]))
    # 按 repetition/phase 放入等宽窗口，只平移事件起点，不拉伸时间或拼接回中。
    for (axis, amplitude), traces in sorted(plot_groups.items()):
        keys = sorted({segment['key'] for trace in traces for segment in trace['segments']})
        duration = max(segment['duration'] for trace in traces for segment in trace['segments'])
        slot = duration*1.12
        for zoom in (False, True):
            fig, axes = plt.subplots(len(traces), 1, figsize=(15, 2.6*len(traces)+.8),
                                     sharex=True, sharey=True, squeeze=False)
            for ax, trace in zip(axes[:, 0], traces):
                if zoom:
                    for index, segment in enumerate(trace['segments']):
                        offset = keys.index(segment['key'])*slot
                        ax.step(offset+segment['setpoint_t'], segment['setpoint'], where='post',
                                color='#64748b', linestyle='--', linewidth=1.3,
                                label='PX4 setpoint' if index == 0 else None)
                        ax.plot(offset+segment['actual_t'], segment['actual'], color='#ea580c',
                                linewidth=1.1, label='Actual' if index == 0 else None)
                        ax.axvline(offset, color='gray', linewidth=.6, alpha=.3)
                    ax.set_xlim(0, (len(keys)-1)*slot+duration)
                else:
                    ax.step(trace['setpoint_t'], trace['setpoint'], where='post',
                            color='#64748b', linestyle='--', linewidth=1.3, label='PX4 setpoint')
                    ax.plot(trace['actual_t'], trace['actual'], color='#ea580c',
                            linewidth=1.1, label='Actual')
                # 使用该 trial 的 FCU 读回值，避免候选倍率或后续配置修改造成歧义。
                parameters = trace['parameters']
                labels = []
                for name in ('P', 'I', 'D', 'FF', 'K'):
                    value = parameters.get('MC_%sRATE_%s' % (axis.upper(), name))
                    labels.append('%s=%s' % (name, format(value, '.6g') if value is not None else 'N/A'))
                ax.set_title('%s | trial %03d | %s' %
                             (trace['candidate_id'], trace['trial_id'], '  '.join(labels)),
                             loc='left', fontsize=11)
                ax.set_ylabel('FRD rate (rad/s)')
                ax.grid(True, alpha=.2)
                ax.axhline(0, color='gray', linewidth=.6, alpha=.4)
                ax.margins(x=0)
            axes[0, 0].legend(loc='upper right', ncol=2)
            if zoom:
                axes[-1, 0].set_xticks([i*slot for i in range(len(keys))])
                axes[-1, 0].set_xticklabels(['Pulse %d' % (i+1) for i in range(len(keys))])
                axes[-1, 0].set_xlabel('Each pulse starts at t=0 in its slot | slot window %.0f ms | recovery omitted' % (duration*1000))
            else:
                axes[-1, 0].set_xlabel('Time since trial start (s)')
            fig.suptitle('%s | excitation amplitude %g deg/s' % (axis.capitalize(), amplitude))
            fig.tight_layout(rect=(0, 0, 1, .97))
            fig.savefig(output / ('comparison_%s_%gdeg_s%s.png' % (axis, amplitude, '_zoom' if zoom else '')), dpi=160)
            plt.close(fig)
    if not rows:
        raise ValueError('no completed pulse trials could be analyzed; check events.yaml')
    summary = summarize(rows)
    differences = []
    for pos in summary:
        if pos['direction'] != 'positive':
            continue
        neg = next((n for n in summary if n['candidate_id'] == pos['candidate_id'] and
                    n['amplitude_deg_s'] == pos['amplitude_deg_s'] and n['direction'] == 'negative'), None)
        if neg:
            differences.append(dict(candidate_id=pos['candidate_id'], amplitude_deg_s=pos['amplitude_deg_s'],
                                    positive_minus_negative={k: v['median']-neg[k]['median']
                                    for k, v in pos.items() if isinstance(v, dict) and k in neg}))
    report = dict(pulses=rows, summary=summary, direction_differences=differences, excluded_trials=skipped,
                  alignment_max_median_error_rad_s=max(alignment) if alignment else None,
                  clock_offset_override_s=args.ros_minus_px4_s, clock_alignment=clock_info, hf_cutoff_hz=args.hf_hz,
                  ulog=str(Path(args.ulog).resolve()), session=str(session.resolve()),
                  note='No ranking or parameter writeback. Recenter is excluded from pulse metrics. Null means unobserved/not measurable. Event-to-FCU transport and sampling delays affect millisecond metrics.')
    (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    write_summary_tables(rows, output / 'summary.csv')
    print('Analyzed %d pulses; generated %d amplitude comparison plots.' % (len(rows), len(list(output.glob('*.png')))))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        print('Analysis failed: '+str(error), file=sys.stderr)
        sys.exit(1)
