#!/usr/bin/env python3
import sys
import re

distance_pattern = re.compile(r"I \((\d+)\) crater_fw: HC-SR04 distance: ([\d.]+) cm")
velocity_pattern = re.compile(
    r"I \((\d+)\) crater_fw: Delta P: ([\d.]+) cm, Velocity: ([\d.]+) cm/s"
)

distances = []
velocities = []

for line in sys.stdin:
    d_match = distance_pattern.search(line)
    if d_match:
        timestamp = int(d_match.group(1))
        distance = float(d_match.group(2))
        distances.append((timestamp, distance))
        continue

    v_match = velocity_pattern.search(line)
    if v_match:
        timestamp = int(v_match.group(1))
        delta = float(v_match.group(2))
        velocity = float(v_match.group(3))
        velocities.append((timestamp, delta, velocity))

print("timestamp_ms,distance_cm")
for ts, dist in distances:
    print(f"{ts},{dist:.2f}")

print("\n--- VELOCITY ---\n")

print("timestamp_ms,delta_cm,velocity_cm_s")
for ts, delta, vel in velocities:
    print(f"{ts},{delta:.2f},{vel:.2f}")
