import csv
import os
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smart_servo_ros.logging_utils import write_row, read_rows


def test_csv_round_trip():
    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, "log.csv")
        write_row(path, {"time": 0.1, "servo_deg": 10.0, "door_deg": 3.75, "torque": 12.0})
        write_row(path, {"time": 0.2, "servo_deg": 20.0, "door_deg": 7.5, "torque": 20.0})

        rows = read_rows(path)

        assert len(rows) == 2
        assert rows[0]["servo_deg"] == "10.0"
        assert rows[1]["torque"] == "20.0"
