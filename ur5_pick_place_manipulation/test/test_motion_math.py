#!/usr/bin/env python3
"""
test_motion_math.py

Unit tests for ur5_pick_place_manipulation/motion_math.py. Pure Python,
no ROS running (no rclpy.init, no node, no simulator) — that is the whole
point of motion_math.py existing as its own module instead of methods on
PickPlaceNode.

Written as unittest.TestCase rather than plain pytest-style functions:
colcon's ament_python test task picks between a plain-unittest step and a
pytest step by checking setup.py's own tests_require list (colcon_core's
PytestPythonTestingStep.match(), not anything in package.xml); without
tests_require=['pytest'] there it silently fell back to running
"python3 -m unittest -v" against this package, which found zero tests
(confirmed by reading its command.log). With tests_require=['pytest'] now
set, colcon runs this through pytest instead (which also produces the
JUnit XML colcon test-result actually aggregates — plain unittest
produces none). Kept the TestCase style anyway since pytest collects it
natively too and it stays runnable as a plain fallback either way; the
assertion style is the only thing that would differ.
"""

import unittest

from ur5_pick_place_manipulation import motion_math


class TestComputePlaceHeight(unittest.TestCase):

    def test_sums_the_four_terms(self):
        # cube_z + half_cube + half_cylinder + clearance, see
        # pick_place_node.py's PLACE_TARGET_MODEL comment for where these
        # numbers come from (pick_place.sdf's red_box / blue_cylinder sizes).
        height = motion_math.compute_place_height(
            cube_z=0.525, cube_half_height=0.025, cylinder_half_length=0.030, clearance=0.002
        )
        self.assertAlmostEqual(height, 0.582)

    def test_zero_clearance(self):
        height = motion_math.compute_place_height(
            cube_z=1.0, cube_half_height=0.1, cylinder_half_length=0.2, clearance=0.0
        )
        self.assertAlmostEqual(height, 1.3)


class TestPathLength(unittest.TestCase):

    def test_zero_for_identical_start_and_goal(self):
        joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        self.assertAlmostEqual(motion_math.path_length(joints, joints), 0.0)

    def test_single_joint_is_plain_difference(self):
        start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.assertAlmostEqual(motion_math.path_length(start, goal), 1.0)

    def test_is_euclidean_across_joints(self):
        # 3-4-5 triangle, generalized: two joints moving 3 and 4 rad should
        # give a combined distance of 5, not 7 (which a sum instead of a
        # norm would give).
        start = [0.0, 0.0]
        goal = [3.0, 4.0]
        self.assertAlmostEqual(motion_math.path_length(start, goal), 5.0)


class TestSplineDuration(unittest.TestCase):

    def test_floors_a_tiny_move(self):
        start = [0.0] * 6
        goal = [0.001] * 6
        duration = motion_math.spline_duration(
            start, goal, max_velocity=0.6283, max_accel=1.0, min_duration=1.0
        )
        self.assertAlmostEqual(duration, 1.0)

    def test_scales_up_for_a_larger_displacement(self):
        start = [0.0] * 6
        small_goal = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        large_goal = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        max_velocity, max_accel = 0.6283, 1.0

        small_duration = motion_math.spline_duration(
            start, small_goal, max_velocity, max_accel, 0.1)
        large_duration = motion_math.spline_duration(
            start, large_goal, max_velocity, max_accel, 0.1)

        self.assertGreater(large_duration, small_duration)

    def test_respects_velocity_and_accel_limits(self):
        # Build the actual spline at the computed duration and check its
        # real sampled peak velocity/acceleration never exceed the budget,
        # the same way spline_duration itself verifies before returning —
        # this test checks the public contract, not the implementation.
        start = [0.0] * 6
        goal = [2.3, -1.1, 0.7, 0.0, 0.0, 0.0]
        max_velocity, max_accel = 0.6283, 1.0

        duration = motion_math.spline_duration(start, goal, max_velocity, max_accel, 1.0)
        samples = motion_math.build_spline_samples(start, goal, duration, steps=200)

        peak_velocity = max(abs(v) for _, _, velocities, _ in samples for v in velocities)
        peak_accel = max(abs(a) for _, _, _, accelerations in samples for a in accelerations)

        self.assertLessEqual(peak_velocity, max_velocity * 1.001)
        self.assertLessEqual(peak_accel, max_accel * 1.001)


class TestBuildSplineSamples(unittest.TestCase):

    def test_starts_and_ends_at_the_right_positions(self):
        start = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        goal = [0.6, 0.1, -0.2, 0.3, -0.4, 0.5]
        duration = 2.0

        samples = motion_math.build_spline_samples(start, goal, duration, steps=20)

        first_t, first_positions, _, _ = samples[0]
        last_t, last_positions, _, _ = samples[-1]

        self.assertAlmostEqual(first_t, 0.0)
        self.assertAlmostEqual(last_t, duration)
        for actual, expected in zip(first_positions, start):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(last_positions, goal):
            self.assertAlmostEqual(actual, expected)

    def test_has_zero_velocity_at_both_ends(self):
        start = [0.0] * 6
        goal = [1.5, -0.8, 0.4, 0.0, 0.2, -0.1]
        duration = 3.0

        samples = motion_math.build_spline_samples(start, goal, duration, steps=20)
        _, _, first_velocities, _ = samples[0]
        _, _, last_velocities, _ = samples[-1]

        for v in first_velocities:
            self.assertAlmostEqual(v, 0.0, places=9)
        for v in last_velocities:
            self.assertAlmostEqual(v, 0.0, places=9)

    def test_returns_requested_number_of_points(self):
        start = [0.0] * 6
        goal = [1.0] * 6
        samples = motion_math.build_spline_samples(start, goal, duration=1.0, steps=40)
        # steps + 1, same convention as pick_place_node's SPLINE_STEPS
        self.assertEqual(len(samples), 41)


if __name__ == '__main__':
    unittest.main()
