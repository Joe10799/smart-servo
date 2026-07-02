import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smart_servo_ros.keyboard_node import key_to_cmd


def test_arrow_keys_map_to_motion_commands():
    assert key_to_cmd("LEFT") == "L"
    assert key_to_cmd("RIGHT") == "R"
    assert key_to_cmd("UP") == "S"
    assert key_to_cmd("DOWN") == "S"


def test_speed_and_control_keys_map_correctly():
    assert key_to_cmd("+") == "+"
    assert key_to_cmd("-") == "-"
    assert key_to_cmd("c") == "CAL"
    assert key_to_cmd("r") == "RESET"
    assert key_to_cmd("q") == "QUIT"
