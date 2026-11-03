#!/usr/bin/env python3
"""在独立 ROS master 上模拟落地和 MAVROS FTP，验证自动下载入口。"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    processes = []
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        with (root/'ros.log').open('w') as log:
            try:
                processes.append(subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log))
                import rosgraph
                import rospy
                from mavros_msgs.msg import State, ExtendedState, FileEntry
                from mavros_msgs.srv import (FileList, FileListResponse, FileOpen, FileOpenResponse,
                                            FileRead, FileReadResponse, FileClose, FileCloseResponse)
                def wait_for(predicate, timeout=15):
                    deadline = time.monotonic()+timeout
                    while not predicate():
                        if time.monotonic() > deadline:
                            raise AssertionError((root/'ros.log').read_text()[-5000:])
                        time.sleep(.05)
                wait_for(rosgraph.is_master_online)
                rospy.init_node('ulog_download_mock', disable_signals=True)
                files, operations = {}, []
                state = State(connected=True, armed=False, mode='POSCTL')
                landed = ExtendedState(landed_state=1)
                state_pub = rospy.Publisher('/mavros/state', State, queue_size=1)
                land_pub = rospy.Publisher('/mavros/extended_state', ExtendedState, queue_size=1)
                def feedback(_):
                    state_pub.publish(state)
                    land_pub.publish(landed)
                timer = rospy.Timer(rospy.Duration(.1), feedback)
                def listing(req):
                    operations.append('list')
                    assert not state.armed
                    return FileListResponse(
                        [FileEntry(name=name, type=0, size=len(data)) for name, data in files.items()], True, 0)
                def opening(req):
                    assert req.mode == 0 and not state.armed
                    operations.append('open')
                    return FileOpenResponse(len(files[Path(req.file_path).name]), True, 0)
                def reading(req):
                    assert not state.armed
                    operations.append('read')
                    return FileReadResponse(files[Path(req.file_path).name][req.offset:req.offset+req.size], True, 0)
                servers = [rospy.Service('/mavros/ftp/list', FileList, listing),
                           rospy.Service('/mavros/ftp/open', FileOpen, opening),
                           rospy.Service('/mavros/ftp/read', FileRead, reading),
                           rospy.Service('/mavros/ftp/close', FileClose, lambda req: FileCloseResponse(True, 0))]
                output = root/'session_px4'
                processes.append(subprocess.Popen(['rosrun', 'holydrone_autotune', 'download_flight_ulog.py',
                    '_output:='+str(output)], stdout=log, stderr=log,
                    env=dict(os.environ, PYTHONUNBUFFERED='1')))
                wait_for(lambda: 'ULog preflight inventory ready' in (root/'ros.log').read_text())
                assert 'open' not in operations and not output.exists()
                state.armed = True
                time.sleep(.4)
                landed.landed_state = 2
                files['flight.ulg'] = b'ULog\x01\x12\x35'+b'x'*40000
                time.sleep(.4)
                assert 'open' not in operations
                landed.landed_state = 1
                state.armed = False
                target = output/'log/flight_001/flight.ulg'
                wait_for(target.exists)
                assert target.read_bytes() == files['flight.ulg']
                time.sleep(.5)
                assert operations.count('open') == 1
                assert not list(output.rglob('*.part'))
                timer.shutdown()
                print('PASS: preflight inventory, no in-flight transfer, landed disarm downloads exact ULog once')
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()


if __name__ == '__main__':
    main()
