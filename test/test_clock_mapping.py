"""実機を開かず、初回配送遅延が時計変換へ混入しないことを検証する。"""
from datetime import timedelta
import pytest
from depthai_driver.clock_mapping import ClockMapping, ClockMonitor, timedelta_ns


def test_legacy_preserves_old_delay():
    mapper = ClockMapping('legacy_first_message', lambda: 10_200_000_000, lambda: 0)
    assert mapper.convert(10_000_000_000) == 10_200_000_000
    assert mapper.convert(11_000_000_000) == 11_200_000_000


def test_host_clock_ignores_old_packet_age():
    mapper = ClockMapping('host_clock', lambda: 110_200_000_000, lambda: 10_200_000_000)
    assert mapper.convert(10_000_000_000) == 110_000_000_000
    assert mapper.convert(11_000_000_000) == 111_000_000_000
    assert mapper.reference_span_ns == 0


def test_minimum_bracket_selected():
    values = iter([0, 100, 200, 220] + [300, 400]*7)
    mapper = ClockMapping('host_clock', lambda: 1000, lambda: next(values))
    assert mapper.convert(0) == 790
    assert mapper.reference_span_ns == 20


def test_invalid_mode():
    with pytest.raises(ValueError):
        ClockMapping('typo', lambda: 0, lambda: 0)


def test_integer_timedelta():
    assert timedelta_ns(timedelta(days=1, seconds=2, microseconds=3)) == 86402000003000


def test_monitor_warns_without_changing_headers():
    ros = [1000]
    mapper = ClockMapping('host_clock', lambda: ros[0], lambda: 0)
    mapper.convert(0)
    monitor = ClockMonitor(mapper, 100, 10)
    assert monitor.check()['clock_monitor_state'] == 'OK'
    ros[0] += 200
    result = monitor.check()
    assert result['clock_monitor_state'] == 'OFFSET_EXCEEDED'
    assert result['clock_offset_delta_ns'] == 200
    assert mapper.convert(10) == 1010
    ros[0] = 1000
    result = monitor.check()
    assert result['clock_monitor_state'] == 'OK'
    assert result['clock_monitor_exceeded_count'] == 1
    assert result['clock_monitor_max_abs_delta_ns'] == 200


def test_monitor_negative_jump_and_threshold_boundary():
    ros = [1000]
    mapper = ClockMapping('host_clock', lambda: ros[0], lambda: 0)
    mapper.convert(0)
    monitor = ClockMonitor(mapper, 100, 10)
    ros[0] = 900
    assert monitor.check()['clock_monitor_state'] == 'OK'
    ros[0] = 899
    assert monitor.check()['clock_monitor_state'] == 'OFFSET_EXCEEDED'


def test_monitor_unreliable_reference_is_not_offset_alarm():
    mapper = ClockMapping('host_clock', lambda: 1000, lambda: 0)
    mapper.convert(0)
    value = [0]
    def delayed_host():
        value[0] += 100
        return value[0]
    mapper.host_now_ns = delayed_host
    monitor = ClockMonitor(mapper, 10, 50)
    result = monitor.check()
    assert result['clock_monitor_state'] == 'REFERENCE_UNRELIABLE'
    assert result['clock_offset_delta_ns'] is None
    assert result['clock_monitor_exceeded_count'] == 0
    assert mapper.offset_ns == 1000


def test_monitor_before_initialization_and_bad_threshold():
    mapper = ClockMapping('host_clock', lambda: 0, lambda: 0)
    assert ClockMonitor(mapper, 100, 10).check()['clock_monitor_state'] == 'NOT_INITIALIZED'
    assert mapper.offset_ns is None
    with pytest.raises(ValueError):
        ClockMonitor(mapper, 0, 10)


def test_monitor_clock_read_failure_keeps_mapping():
    mapper = ClockMapping('host_clock', lambda: 1000, lambda: 0)
    mapper.convert(0)
    def failing_host():
        raise RuntimeError('clock unavailable')
    mapper.host_now_ns = failing_host
    result = ClockMonitor(mapper, 100, 10).check()
    assert result['clock_monitor_state'] == 'REFERENCE_UNRELIABLE'
    assert result['clock_monitor_unreliable_count'] == 1
    assert mapper.offset_ns == 1000
