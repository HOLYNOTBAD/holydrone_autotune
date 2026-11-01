#!/usr/bin/env python3
"""Join ROS experiment events to PX4 ULog using recorded MAVROS TIMESYNC."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import yaml
from holydrone_autotune.autotune_analysis import pulse_metrics, summarize


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--session', required=True, help='directory containing events.yaml and timesync.csv')
    p.add_argument('--ulog', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--ros-minus-px4-s', type=float, help='explicit measured clock offset; overrides TIMESYNC')
    p.add_argument('--hf-hz', type=float, default=30)
    args = p.parse_args()
    if args.hf_hz <= 0:
        p.error('--hf-hz must be positive')
    from pyulog import ULog
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    session, output = Path(args.session), Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    events = list(yaml.safe_load_all((session / 'events.yaml').read_text()))
    sync = []
    if args.ros_minus_px4_s is None:
        with (session / 'timesync.csv').open(newline='') as stream:
            sync = [(float(row['ros_s']), float(row['ros_minus_px4_s']), float(row['round_trip_time_ms']))
                    for row in csv.DictReader(stream)]
    sync = np.asarray(sync, dtype=float)
    if args.ros_minus_px4_s is None and (len(sync) < 10 or not np.isfinite(sync).all()):
        raise ValueError('need recorded TIMESYNC samples or explicit --ros-minus-px4-s')
    if args.ros_minus_px4_s is None and np.any(np.diff(sync[:, 0]) <= 0):
        raise ValueError('ROS clock is not monotonic in TIMESYNC; split the session')
    def px4_time(event):
        stamp = event['timestamp']
        if args.ros_minus_px4_s is not None:
            return stamp-args.ros_minus_px4_s
        # Local interpolation follows slow timesync drift; refuse stale extrapolation.
        if stamp < sync[0, 0] or stamp > sync[-1, 0] or np.min(np.abs(sync[:, 0]-stamp)) > 2:
            raise ValueError('TIMESYNC does not cover trial event')
        return stamp-float(np.interp(stamp, sync[:, 0], sync[:, 1]))
    ulog = ULog(args.ulog)
    def dataset(name):
        data = ulog.get_dataset(name).data
        t = np.asarray(data['timestamp'], dtype=float)*1e-6
        if len(t) < 2 or np.any(np.diff(t) <= 0):
            raise ValueError('invalid timestamps: '+name)
        return t, data
    ta, actual = dataset('vehicle_angular_velocity')
    ts, setpoint = dataset('vehicle_rates_setpoint')
    tt, torque = dataset('vehicle_torque_setpoint')
    tc, allocator = dataset('control_allocator_status')
    for topic in ['vehicle_attitude', 'vehicle_local_position', 'battery_status', 'vehicle_status']:
        dataset(topic)  # Explicitly report missing required ULog streams.
    trials, rows, skipped, alignment = {}, [], [], []
    for e in events:
        if e and e.get('trial_id', 0):
            trials.setdefault(e['trial_id'], []).append(e)
    for trial_id, ev in trials.items():
        starts = [e for e in ev if e['event_type'] == 'TRIAL_START']
        ends = [e for e in ev if e['event_type'] in ('TRIAL_FINISHED', 'TRIAL_ABORTED')]
        if not starts or not ends or ends[0]['event_type'] != 'TRIAL_FINISHED':
            skipped.append(dict(trial_id=trial_id, reason='aborted/incomplete')); continue
        start, end = px4_time(starts[0]), px4_time(ends[0])
        axis = starts[0]['axis']; idx = ['roll', 'pitch', 'yaw'].index(axis)
        for times in [ta, ts, tt, tc]:
            if times[0] > start or times[-1] < end:
                raise ValueError('ULog does not cover trial; verify selected flight and clock offset')
        pulses = [e for e in ev if e['event_type'] in ('PULSE_POSITIVE', 'PULSE_NEGATIVE', 'PULSE_ZERO')]
        boundaries = [(px4_time(e), e) for e in pulses] + [(end, ends[0])]
        av = np.asarray(actual['xyz[%d]' % idx]); sp = np.asarray(setpoint[axis])
        tq = np.asarray(torque['xyz[%d]' % idx])
        unallocated = np.abs(np.asarray(allocator['unallocated_torque[%d]' % idx])) > 1e-5
        for j, (begin, e) in enumerate(boundaries[:-1]):
            if e['event_type'] == 'PULSE_ZERO':
                continue
            finish, following = boundaries[j+1]
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
            metrics = pulse_metrics(ta[mask], av[mask], target, ta[rm], av[rm], args.hf_hz)
            # Zero-order hold integration over the whole pulse, including its boundaries.
            inner = tc[(tc > begin) & (tc < finish)]
            grid = np.r_[begin, inner, finish]
            held = unallocated[np.clip(np.searchsorted(tc, grid[:-1], side='right')-1, 0, len(tc)-1)]
            metrics['allocator_unachieved_s'] = float(np.sum(np.diff(grid)*held))
            rows.append(dict(candidate_id=e['candidate_id'], trial_id=trial_id,
                             amplitude_deg_s=e['amplitude_deg_s'], repetition=e['repetition'],
                             phase=e['phase'], direction='positive' if sign > 0 else 'negative', **metrics))
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        am, sm, tm = [(t >= start) & (t <= end) for t in [ta, ts, tt]]
        axes[0].plot(ts[sm]-start, sp[sm], label='PX4 setpoint')
        axes[0].plot(ta[am]-start, av[am], label='actual')
        axes[0].set_ylabel('FRD rad/s'); axes[0].legend()
        # Setpoint is held between samples, never linearly ramped across steps.
        held = sp[np.clip(np.searchsorted(ts, ta[am], side='right')-1, 0, len(ts)-1)]
        axes[1].plot(ta[am]-start, held-av[am]); axes[1].set_ylabel('error rad/s')
        axes[2].plot(tt[tm]-start, tq[tm]); axes[2].set_ylabel('normalized torque'); axes[2].set_xlabel('s')
        fig.suptitle('%s / %s / trial %d' % (starts[0]['candidate_id'], axis, trial_id))
        fig.tight_layout(); fig.savefig(output / ('trial_%03d.png' % trial_id)); plt.close(fig)
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
                  clock_offset_override_s=args.ros_minus_px4_s, hf_cutoff_hz=args.hf_hz,
                  ulog=str(Path(args.ulog).resolve()), session=str(session.resolve()),
                  note='No ranking or parameter writeback. Null means unobserved/not measurable.')
    (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('Wrote %d pulse metrics and %d trial plots to %s' % (len(rows), len(list(output.glob('*.png'))), output))


if __name__ == '__main__':
    main()
