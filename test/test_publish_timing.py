"""ROS・cameraを起動せず、診断proxyの委譲と有界保存を確認する。"""
from depthai_driver.publish_timing import TimingBuffer, TimedPublisher
import pytest
import csv
import json
from pathlib import Path
import signal
import subprocess
import sys
from depthai_driver import publish_timing


@pytest.mark.parametrize('capacity', [0, -1, True, 1.5])
def test_invalid_capacity(tmp_path, capacity):
    with pytest.raises(ValueError):
        TimingBuffer(tmp_path/'timing.csv', capacity)


def test_bounded_save(tmp_path):
    buffer = TimingBuffer(tmp_path/'timing.csv', 2)
    for i in range(3):
        buffer.add('poll', i, .001)
    assert len(buffer.rows) == 2
    assert buffer.evicted == 1
    buffer.save()
    assert 'duration_ms' in (tmp_path/'timing.csv').read_text()
    marker = json.loads((tmp_path/'timing.csv.complete.json').read_text())
    assert marker['saved_rows'] == 2
    assert marker['evicted'] == 1


def test_delegation(tmp_path):
    class Publisher:
        topic_name = '/test'
        def publish(self, message):
            self.message = message
    publisher = Publisher()
    buffer = TimingBuffer(tmp_path/'timing.csv')
    proxy = TimedPublisher(publisher, buffer)
    proxy.publish(123)
    assert publisher.message == 123
    assert buffer.rows[0][0] == '/test'


def test_handler_restored_on_save_error(tmp_path):
    path = tmp_path/'exists.csv'
    path.write_text('user data')
    original = signal.getsignal(signal.SIGINT)
    with pytest.raises(FileExistsError):
        TimingBuffer(path).save()
    assert signal.getsignal(signal.SIGINT) == original
    assert path.read_text() == 'user data'
    assert not (tmp_path/'exists.csv.complete.json').exists()


def test_repeated_sigint_during_save_is_ignored_only_in_save(tmp_path):
    # 本物のSIGINTを送る試験は子processに隔離し、pytest本体を中断しない。
    script = '''
import csv, os, signal, sys
sys.path.insert(0, sys.argv[1])
from depthai_driver.publish_timing import TimingBuffer
original = signal.getsignal(signal.SIGINT)
real_writer = csv.writer
class Writer:
    def __init__(self, stream): self.delegate = real_writer(stream)
    def writerow(self, row): self.delegate.writerow(row)
    def writerows(self, rows):
        os.kill(os.getpid(), signal.SIGINT)
        self.delegate.writerows(rows)
csv.writer = Writer
buffer = TimingBuffer(sys.argv[2], 100)
for index in range(100): buffer.add('poll', index, .001)
buffer.save()
assert signal.getsignal(signal.SIGINT) == original
'''
    path = tmp_path/'signal.csv'
    subprocess.run([sys.executable, '-c', script,
                    str(Path(publish_timing.__file__).parents[1]), str(path)], check=True, timeout=10)
    with path.open() as stream:
        assert len(list(csv.DictReader(stream))) == 100
    assert json.loads((tmp_path/'signal.csv.complete.json').read_text())['saved_rows'] == 100
