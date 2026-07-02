import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smart_servo_ros.tcp_bridge import parse_state_line


def test_parse_state_line_extracts_numbers():
    parsed = parse_state_line("STATE servo=12.34 door=56.78 torque=9.01")

    assert parsed is not None
    assert parsed == (12.34, 56.78, 9.01)


def test_parse_state_line_returns_none_for_unrelated_line():
    assert parse_state_line("SMART_SERVO_READY") is None
