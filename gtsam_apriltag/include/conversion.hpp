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

#include <gtsam/geometry/Pose2.h>

namespace gtsam_apriltag
{

inline auto ToWpiPose(const gtsam::Pose2 & pose) -> wpi::math::Pose2d
{
  return wpi::math::Pose2d{
    wpi::units::meter_t{pose.x()}, wpi::units::meter_t{pose.y()},
    wpi::math::Rotation2d{pose.rotation().matrix()}};
}
inline auto ToGtsamPose(const wpi::math::Pose2d & pose) -> gtsam::Pose2
{
  return gtsam::Pose2{pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value()};
}
inline auto ToGtsamPose(const wpi::math::Transform2d & transform) -> gtsam::Pose2
{
  return gtsam::Pose2{
    transform.X().value(), transform.Y().value(), transform.Rotation().Radians().value()};
}
}  // namespace gtsam_apriltag
