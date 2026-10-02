/*
 * Copyright 2026, The Drop Bears - FRC Team 4774
 * Copyright 2026, The Warriors of East Harlem - FRC Team 1880
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 */

#pragma once

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Transform2d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam_apriltag
{
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::X;

inline auto ToWpi(const gtsam::Pose2 & pose) -> wpi::math::Pose2d
{
  return wpi::math::Pose2d{
    wpi::units::meter_t{pose.x()}, wpi::units::meter_t{pose.y()},
    wpi::math::Rotation2d{pose.rotation().matrix()}};
}
inline auto ToGtsam(const wpi::math::Pose2d & pose) -> gtsam::Pose2
{
  return gtsam::Pose2{pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value()};
}
inline auto ToGtsam(const wpi::math::Pose3d & pose) -> gtsam::Pose3
{
  return gtsam::Pose3(pose.ToMatrix());
}
inline auto ToGtsam(const wpi::math::Rotation2d & rotation) -> gtsam::Rot2
{
  return gtsam::Rot2(rotation.Radians().value());
}
inline auto ToGtsam(const wpi::math::Transform2d & transform) -> gtsam::Pose2
{
  return gtsam::Pose2{
    transform.X().value(), transform.Y().value(), transform.Rotation().Radians().value()};
}
inline auto ToGtsam(const wpi::math::Transform3d & transform) -> gtsam::Pose3
{
  return gtsam::Pose3(transform.ToMatrix());
}
inline auto ToGtsam(const wpi::math::Translation2d & translation) -> gtsam::Point2
{
  return gtsam::Point2(translation.X().value(), translation.Y().value());
}
inline auto ToGtsam(const wpi::math::Translation3d & translation) -> gtsam::Point3
{
  return gtsam::Point3(translation.X().value(), translation.Y().value(), translation.Z().value());
}
inline auto ToGtsam(const wpi::math::Twist2d & twist) -> gtsam::Pose2
{
  return gtsam::Pose2(twist.dx(), twist.dy(), twist.dtheta());
}
inline auto ToGtsam(const CameraCalibration & camera_calibration) -> gtsam::Cal3DS2
{
  return gtsam::Cal3DS2(
    camera_calibration.fx, camera_calibration.fy, camera_calibration.s, camera_calibration.u0,
    camera_calibration.v0, camera_calibration.k1, camera_calibration.k2, camera_calibration.p1,
    camera_calibration.p2);
}
inline auto ToPoseKey(const wpi::units::second_t timestamp)
{
  return X(static_cast<uint64_t>(timestamp.value() * 1e6));
}
inline auto ToBiasKey(const wpi::units::second_t timestamp)
{
  return B(static_cast<uint64_t>(timestamp.value() * 1e6));
}
inline auto ApriltagCorners(const wpi::math::Pose3d & pose) -> std::array<gtsam::Point3, 4>
{
  // FRC tags are 6.5in square
  // Corners numbered counter clockwise from bottom left

  // TODO Cache these calculations
  using wpi::math::Rotation3d;
  using wpi::math::Transform3d;
  using wpi::units::inch_t;
  const std::array<gtsam::Point3, 4> points = {
    ToGtsam(
      pose.TransformBy(Transform3d(inch_t(0.0), inch_t(-6.5 / 2), inch_t(-6.5 / 2), Rotation3d()))
        .Translation()),
    ToGtsam(
      pose.TransformBy(Transform3d(inch_t(0.0), inch_t(6.5 / 2), inch_t(-6.5 / 2), Rotation3d()))
        .Translation()),
    ToGtsam(
      pose.TransformBy(Transform3d(inch_t(0.0), inch_t(6.5 / 2), inch_t(6.5 / 2), Rotation3d()))
        .Translation()),
    ToGtsam(
      pose.TransformBy(Transform3d(inch_t(0.0), inch_t(-6.5 / 2), inch_t(6.5 / 2), Rotation3d()))
        .Translation()),
  };

  return points;
}
}  // namespace gtsam_apriltag
