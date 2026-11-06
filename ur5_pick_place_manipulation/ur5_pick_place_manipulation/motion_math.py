#!/usr/bin/env python3
"""
motion_math.py

Pure, ROS-free math used by pick_place_node.py: no rclpy, no live node, no
simulator. Kept separate specifically so it can be unit tested without
anything ROS running (see test/test_motion_math.py).

compute_place_height, path_length, spline_duration, and build_spline_samples
used to live as methods or inline expressions inside PickPlaceNode; moved
here unchanged in behavior, parameterized explicitly instead of reading
module-level constants directly, which is what actually makes them testable
in isolation.
"""

from scipy.interpolate import CubicSpline


def compute_place_height(
    cube_z: float, cube_half_height: float, cylinder_half_length: float, clearance: float
) -> float:
    """Z for the cylinder's center so it rests centered on top of the cube."""
    return cube_z + cube_half_height + cylinder_half_length + clearance


def path_length(start: list, goal: list) -> float:
    """Straight-line distance in joint space (radians), unweighted across joints."""
    return sum((g - s) ** 2 for s, g in zip(start, goal)) ** 0.5


def spline_duration(
    start: list,
    goal: list,
    max_velocity: float,
    max_accel: float,
    min_duration: float,
) -> float:
    """
    Duration long enough that a per-joint cubic spline (zero velocity at
    both ends) from start to goal stays within max_velocity and max_accel,
    with a min_duration floor.

    For a single cubic segment clamped to zero velocity at both ends, peak
    velocity = 1.5 * displacement / T (standard result for this boundary
    condition), which gives a first estimate of T from the velocity budget
    alone. That estimate is not just trusted for acceleration too: a trial
    spline is built at that duration and its actual second derivative is
    sampled, since what matters is what the spline really does, not just
    the analytical formula. Acceleration scales as 1/T^2 for a fixed
    displacement, so if the sampled peak exceeds the budget, this scales T
    up by exactly the factor needed rather than guessing and re-checking.
    """
    max_disp = max(abs(g - s) for s, g in zip(start, goal))
    duration = max(min_duration, 1.5 * max_disp / max_velocity)

    trial_splines = [
        CubicSpline([0.0, duration], [s, g], bc_type=((1, 0.0), (1, 0.0)))
        for s, g in zip(start, goal)
    ]
    sample_times = [duration * i / 100.0 for i in range(101)]
    peak_accel = max(
        abs(spline(t, 2)) for spline in trial_splines for t in sample_times
    )
    if peak_accel > max_accel:
        duration *= (peak_accel / max_accel) ** 0.5

    return max(duration, min_duration)


def build_spline_samples(start: list, goal: list, duration: float, steps: int):
    """
    Sample a per-joint cubic spline (zero velocity at both ends) from start
    to goal over [0, duration] at `steps + 1` evenly spaced times.

    Returns a list of (t, positions, velocities, accelerations) tuples, one
    per sample, each a plain float / list of floats — no ROS message types,
    so this can be compared directly in a test. pick_place_node.py wraps
    this into a RobotTrajectory; the wrapping itself is not pure (it
    constructs ROS messages) and is not duplicated here.
    """
    splines = [
        CubicSpline([0.0, duration], [s, g], bc_type=((1, 0.0), (1, 0.0)))
        for s, g in zip(start, goal)
    ]

    samples = []
    for i in range(steps + 1):
        t = duration * i / steps
        positions = [float(s(t)) for s in splines]
        velocities = [float(s(t, 1)) for s in splines]
        accelerations = [float(s(t, 2)) for s in splines]
        samples.append((t, positions, velocities, accelerations))
    return samples
