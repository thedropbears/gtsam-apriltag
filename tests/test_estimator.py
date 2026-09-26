import random

import robotpy_fields
import wpimath

import gtsam_apriltag


def test_observation_at_beginning():
    field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
    e = gtsam_apriltag.Estimator(field)

    e.update(wpimath.Pose2d(1, 1, 0), 1.0)
    e.update(wpimath.Pose2d(2, 2, 0), 2.0)

    for i in range(1, 4):
        tag_pose = field.get_tag_pose(i)
        obs = tag_pose - wpimath.Pose3d(10, 10, 0, wpimath.Rotation3d())
        e.add_observation(1.0, i, obs, wpimath.Transform3d())

    e.update(wpimath.Pose2d(2, 2, 0), 3.0)

    final_estimate = e.get_pose()
    assert abs(11.0 - final_estimate.x) < 0.01
    assert abs(11.0 - final_estimate.y) < 0.01


def test_observation_in_middle():
    field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
    e = gtsam_apriltag.Estimator(field)

    e.update(wpimath.Pose2d(1, 1, 0), 1.0)
    e.update(wpimath.Pose2d(2, 2, 0), 2.0)

    for i in range(1, 4):
        tag_pose = field.get_tag_pose(i)
        obs = tag_pose - wpimath.Pose3d(10, 10, 0, wpimath.Rotation3d())
        e.add_observation(1.5, i, obs, wpimath.Transform3d())

    e.update(wpimath.Pose2d(2, 2, 0), 3.0)

    final_estimate = e.get_pose()
    assert abs(10.5 - final_estimate.x) < 0.01
    assert abs(10.5 - final_estimate.y) < 0.01


def test_observation_at_end():
    field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
    e = gtsam_apriltag.Estimator(field)

    e.update(wpimath.Pose2d(1, 1, 0), 1.0)
    e.update(wpimath.Pose2d(2, 2, 0), 2.0)

    for i in range(1, 4):
        tag_pose = field.get_tag_pose(i)
        obs = tag_pose - wpimath.Pose3d(10, 10, 0, wpimath.Rotation3d())
        e.add_observation(2.0, i, obs, wpimath.Transform3d())

    e.update(wpimath.Pose2d(2, 2, 0), 3.0)

    final_estimate = e.get_pose()
    assert abs(10.0 - final_estimate.x) < 0.01
    assert abs(10.0 - final_estimate.y) < 0.01


def test_observation_before_odom():
    field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
    e = gtsam_apriltag.Estimator(field)

    e.update(wpimath.Pose2d(1, 1, 0), 1.0)
    e.update(wpimath.Pose2d(2, 2, 0), 2.0)

    for i in range(1, 4):
        tag_pose = field.get_tag_pose(i)
        obs = tag_pose - wpimath.Pose3d(10, 10, 0, wpimath.Rotation3d())
        e.add_observation(3.0, i, obs, wpimath.Transform3d())

    e.update(wpimath.Pose2d(2, 2, 0), 3.0)

    final_estimate = e.get_pose()
    assert abs(10.0 - final_estimate.x) < 0.01
    assert abs(10.0 - final_estimate.y) < 0.01


def clamped_gaussian(mu, sigma, multiplier):
    val = random.gauss(mu, sigma)
    clamped_lower = max(-multiplier * sigma, val)
    clamped = min(multiplier * sigma, clamped_lower)
    return clamped


def test_noisy_observations():
    field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
    e = gtsam_apriltag.Estimator(field)

    e.update(wpimath.Pose2d(1, 1, 0), 1.0)
    e.update(wpimath.Pose2d(2, 2, 0), 2.0)

    sigma = 0.1
    # Statistical so run a bunch of times
    # This would happen when the robot is stationary waiting for enable
    # so it's not unrealistic to have this happen
    # Simulate 1s at 50Hz
    for n in range(50):
        for i in range(1, 4):
            tag_pose = field.get_tag_pose(i)
            obs = tag_pose - wpimath.Pose3d(
                10 + clamped_gaussian(0.0, sigma, 3.0),
                10 + clamped_gaussian(0.0, sigma, 3.0),
                0,
                wpimath.Rotation3d(),
            )
            e.add_observation(3.0 + n * 0.02, i, obs, wpimath.Transform3d())

        e.update(wpimath.Pose2d(2, 2, 0), 3.0 + n * 0.02)

    final_estimate = e.get_pose()
    assert abs(10.0 - final_estimate.x) < 2 * sigma
    assert abs(10.0 - final_estimate.y) < 2 * sigma


def test_floating_tags():
    field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
    e = gtsam_apriltag.Estimator(field)

    floating_tags = {
        25: wpimath.Pose3d(10, 0, 0, wpimath.Rotation3d()),
        30: wpimath.Pose3d(0, 10, 0, wpimath.Rotation3d()),
        35: wpimath.Pose3d(0, 0, 0, wpimath.Rotation3d()),
    }

    e.set_floating_tag_ids(floating_tags.keys())

    robot_pose = wpimath.Pose3d(10, 10, 0, wpimath.Rotation3d())

    # Initialise
    e.update(wpimath.Pose2d(1, 1, 0), 0.0)

    # Use fixed tag pose once to anchor results

    fixed_tag_id = 5
    e.add_observation(
        0.0,
        fixed_tag_id,
        (field.get_tag_pose(fixed_tag_id) - robot_pose),
        wpimath.Transform3d(),
    )
    fixed_tag_id = 6
    e.add_observation(
        0.0,
        fixed_tag_id,
        (field.get_tag_pose(fixed_tag_id) - robot_pose),
        wpimath.Transform3d(),
    )

    for i in range(25):
        for id, tag_pose in floating_tags.items():
            obs = tag_pose - robot_pose
            e.add_observation(0.0 + i * 0.1, id, obs, wpimath.Transform3d(), 0.2)
        e.update(wpimath.Pose2d(1, 1, 0) + wpimath.Transform2d(i, i, 0), 0.0 + i * 0.1)
        robot_pose = robot_pose + wpimath.Transform3d(1, 1, 0, wpimath.Rotation3d())

    # Undo last transform that wasn't added to estimator update
    robot_pose = robot_pose + wpimath.Transform3d(-1, -1, 0, wpimath.Rotation3d())

    final_estimate = e.get_pose()
    # e.print()
    assert abs(robot_pose.x - final_estimate.x) < 0.01
    assert abs(robot_pose.y - final_estimate.y) < 0.01
