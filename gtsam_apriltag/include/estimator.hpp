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

#include "observation.hpp"

#include <wpi/fields/Field.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/interpolation/TimeInterpolatableBuffer.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

#include <queue>

namespace gtsam_apriltag
{
using wpi::math::Pose2d;

class Estimator
{
public:
  Estimator(
    const wpi::fields::Field & field, wpi::units::second_t window_size = wpi::units::second_t{5.0});

  auto AddObservation(
    const wpi::units::second_t timestamp, const int tag_id,
    const wpi::math::Transform3d & camera_to_tag, const wpi::math::Transform3d & base_to_camera,
    const wpi::units::meter_t uncertainty = wpi::units::meter_t{0.05}) -> void;
  auto Update(const Pose2d & odometry, const wpi::units::second_t timestamp = {}) -> Pose2d;
  auto GetPose() const -> Pose2d;
  auto Reset() -> void;
  auto Print() const -> void;
  auto SetOdometryStdDevs(const double x, const double y, const double theta) -> void;

private:
  auto ProcessObservations() -> void;

  const wpi::fields::Field field_;
  gtsam::IncrementalFixedLagSmoother smoother_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values values_;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps_;
  wpi::math::TimeInterpolatableBuffer<Pose2d> interpolator_;
  std::queue<Observation> observations_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;

  Pose2d estimated_pose_;

  Pose2d previous_odom_;
  gtsam::Key previous_odom_key_;
  bool initialised_ = false;
};
}  // namespace gtsam_apriltag
