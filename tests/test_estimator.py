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
