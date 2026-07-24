import csv
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_logs.py <csv-file>")
        return

    path = Path(sys.argv[1])
    if not path.exists():
        print(f"File not found: {path}")
        return

    rows = load_csv(path)
    if not rows:
        print("No data to plot")
        return

    times = [float(r["time"]) - float(rows[0]["time"]) for r in rows]
    servo = [float(r["servo_deg"]) for r in rows]
    door = [float(r["door_deg"]) for r in rows]
    torque = [float(r["torque"]) for r in rows]
    speed = [float(r["speed"]) for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    axes[0].plot(times, servo, label="Servo angle (deg)")
    axes[0].plot(times, door, label="Door angle (deg)")
    axes[0].set_ylabel("Angle (deg)")
    axes[0].legend()

    axes[1].plot(times, torque, color="red", label="Torque")
    axes[1].set_ylabel("Torque")
    axes[1].legend()

    axes[2].plot(times, speed, color="green", label="Speed (deg/s)")
    axes[2].set_ylabel("Speed (deg/s)")
    axes[2].legend()

    axes[2].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
