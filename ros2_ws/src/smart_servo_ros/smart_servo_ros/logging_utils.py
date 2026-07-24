import csv
from pathlib import Path

DEFAULT_FIELDS = ["time", "servo_deg", "door_deg", "torque", "speed", "direction", "calibrated", "detecting"]


def write_row(path, row):
    path = Path(path)
    file_exists = path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=DEFAULT_FIELDS)
        if not file_exists:
            writer.writeheader()
        writer.writerow({field: row.get(field, "") for field in DEFAULT_FIELDS})


def read_rows(path):
    path = Path(path)
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))
