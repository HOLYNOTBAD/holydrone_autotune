#!/usr/bin/env python3
"""落地解除武装后，通过 MAVROS FTP 下载本航次 ULog；不发送飞行控制或参数写入。"""
from pathlib import Path, PurePosixPath
import time


class FlightMonitor:
    def __init__(self):
        self.connected = self.armed = self.airborne = False
        self.state_time = self.land_time = float('-inf')
        self.landed = 0
        self.baseline = None
        self.flight_baseline = None
        self.pending = None
        self.flight = 0

    def state(self, connected, armed, now):
        self.state_time = now
        self.connected = connected
        if not connected:
            return  # 失联不等于解除武装。
        if armed and not self.armed:
            self.flight += 1
            self.flight_baseline = None if self.baseline is None else dict(self.baseline)
            self.baseline = None
            self.pending = None
            self.airborne = False
        elif self.armed and not armed and self.airborne:
            self.pending = dict(baseline=self.flight_baseline, flight=self.flight, disarmed_at=now)
        self.armed = armed

    def extended(self, landed, now):
        self.landed, self.land_time = landed, now
        if self.armed and landed == 2:
            self.airborne = True

    def ground(self, now):
        return (self.connected and not self.armed and self.landed == 1 and
                0 <= now-self.state_time < 3 and 0 <= now-self.land_time < 3)

    def safe(self, now):
        return self.pending is not None and self.ground(now)


class Downloader:
    def __init__(self, ftp, remote, output, check, progress):
        self.ftp, self.remote = ftp, remote.rstrip('/')
        self.output, self.check, self.progress = Path(output), check, progress
        self.completed = {}

    def call(self, operation, **kwargs):
        self.check()
        result = self.ftp[operation](**kwargs)
        if not result.success:
            raise RuntimeError('FTP %s failed: errno %s' % (operation, getattr(result, 'r_errno', '?')))
        self.check()
        return result

    def inventory(self):
        files, todo = {}, [self.remote]
        while todo:
            directory = todo.pop()
            self.check()
            response = self.ftp['list'](dir_path=directory)
            self.check()
            if not response.success:
                if directory == self.remote and response.r_errno == 2:
                    return {}
                raise RuntimeError('FTP list failed: %s errno %s' % (directory, response.r_errno))
            for item in response.list:
                if item.name in ('.', '..'):
                    continue
                if '/' in item.name or '\\' in item.name or not item.name:
                    raise RuntimeError('Invalid FTP entry name')
                name = directory+'/'+item.name
                if item.type == 1:
                    todo.append(name)
                elif item.type == 0 and item.name.lower().endswith('.ulg'):
                    files[name] = item.size
        return files

    def download(self, remote, expected_size, flight):
        relative = PurePosixPath(remote).relative_to(self.remote)
        if '..' in relative.parts:
            raise ValueError('Invalid remote path')
        target = self.output/'log'/('flight_%03d' % flight)/str(relative)
        if target.exists():
            with target.open('rb') as stream:
                valid = stream.read(7) == b'ULog\x01\x12\x35'
            if self.completed.get(target) == (remote, expected_size) and target.stat().st_size == expected_size and valid:
                return target
            raise RuntimeError('Existing output does not match: '+str(target))
        target.parent.mkdir(parents=True, exist_ok=True)
        partial = target.with_suffix('.ulg.part')
        self.check()
        opened = self.ftp['open'](file_path=remote, mode=0)
        if not opened.success:
            raise RuntimeError('FTP open failed: '+remote)
        try:
            self.check()
            if opened.size != expected_size:
                raise RuntimeError('ULog size still changing')
            offset, started, last = 0, time.monotonic(), 0.
            with partial.open('wb') as stream:
                while offset < expected_size:
                    chunk = bytes(self.call('read', file_path=remote, offset=offset,
                                            size=min(239, expected_size-offset)).data)
                    if not chunk or len(chunk) > expected_size-offset:
                        raise RuntimeError('Invalid or empty FTP read')
                    stream.write(chunk)
                    offset += len(chunk)
                    now = time.monotonic()
                    if now-last >= 1 or offset == expected_size:
                        speed = offset/max(now-started, .001)
                        self.progress('ULog %s: %.1f%% (%d/%d bytes), %.1f KiB/s, ETA %.0f s',
                                      relative, 100.*offset/max(expected_size, 1), offset, expected_size,
                                      speed/1024, (expected_size-offset)/max(speed, 1))
                        last = now
            with partial.open('rb') as stream:
                if stream.read(7) != b'ULog\x01\x12\x35':
                    raise RuntimeError('Downloaded file is not a ULog')
            self.check()
        finally:
            # 即使重新解锁也关闭本次 FTP 会话，保留 .part，绝不发布不完整 .ulg。
            closed = self.ftp['close'](file_path=remote)
            if not closed.success:
                raise RuntimeError('FTP close failed')
        self.check()
        partial.replace(target)
        self.completed[target] = (remote, expected_size)
        self.progress('ULog saved: %s (%d bytes)', target, expected_size)
        return target


def main():
    import rospy
    from mavros_msgs.msg import State, ExtendedState
    from mavros_msgs.srv import FileList, FileOpen, FileRead, FileClose
    rospy.init_node('download_flight_ulog')
    monitor = FlightMonitor()
    prefix = rospy.get_param('~mavros_namespace', '/mavros').rstrip('/')
    rospy.Subscriber(prefix+'/state', State,
                     lambda msg: monitor.state(msg.connected, msg.armed, time.monotonic()), queue_size=10)
    rospy.Subscriber(prefix+'/extended_state', ExtendedState,
                     lambda msg: monitor.extended(msg.landed_state, time.monotonic()), queue_size=10)
    def check():
        if rospy.is_shutdown() or not monitor.ground(time.monotonic()):
            raise RuntimeError('FTP paused: require fresh connected, landed, disarmed state')
    ftp = {name: rospy.ServiceProxy(prefix+'/ftp/'+name, service)
           for name, service in [('list', FileList), ('open', FileOpen), ('read', FileRead), ('close', FileClose)]}
    downloader = Downloader(ftp, rospy.get_param('~remote_log_dir', '/fs/microsd/log'),
                            rospy.get_param('~output'), check, rospy.loginfo)
    previous, retry_at, inventory_at = None, 0., 0.
    rospy.loginfo('ULog downloader waiting for ground connection; keep running until download completes')
    while not rospy.is_shutdown():
        now = time.monotonic()
        try:
            if monitor.ground(now) and now >= retry_at:
                pending = monitor.pending
                if pending is None and now >= inventory_at:
                    baseline = downloader.inventory()
                    check()
                    monitor.baseline = baseline
                    inventory_at = now+5
                    rospy.loginfo('ULog preflight inventory ready: %d files', len(baseline))
                elif pending is not None and now-pending['disarmed_at'] >= 2:
                    if pending['baseline'] is None:
                        raise RuntimeError('No preflight inventory: cannot identify this flight; use manual download')
                    files = downloader.inventory()
                    changed = {p: size for p, size in files.items()
                               if size > 0 and pending['baseline'].get(p) != size}
                    if changed and changed == previous:
                        for path, size in sorted(changed.items()):
                            downloader.download(path, size, pending['flight'])
                        if monitor.pending is pending:
                            monitor.pending = None
                            monitor.baseline = files
                            inventory_at = now+5
                            previous = None
                    else:
                        previous = changed
                    retry_at = time.monotonic()+2
        except (RuntimeError, OSError, rospy.ServiceException) as error:
            rospy.logwarn('ULog: %s; retry in 5 s', error)
            retry_at = time.monotonic()+5
        time.sleep(.2)


if __name__ == '__main__':
    main()
