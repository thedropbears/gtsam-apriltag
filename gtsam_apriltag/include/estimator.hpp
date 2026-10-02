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

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/PlanarGyroFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

#include <array>
#include <memory>
#include <queue>
#include <set>
#include <vector>

namespace gtsam_apriltag
{
using wpi::math::Pose2d;
using wpi::math::Rotation2d;
using wpi::math::Twist2d;

class Estimator
{
public:
  Estimator(
    const wpi::fields::Field & field, wpi::units::second_t window_size = wpi::units::second_t{5.0});

  auto AddObservation(const TagObservation & obs) -> void;
  auto AddObservation(const CornersObservation & obs) -> void;
  auto Update(
    const Rotation2d & imu_delta, const Twist2d & robot_delta, const double dt,
    const wpi::units::second_t timestamp = {}) -> Pose2d;
  auto GetPose() const -> Pose2d;
  auto Reset() -> void;
  auto Print() const -> void;
  auto SetOdometryStdDevs(const double x, const double y, const double theta) -> void;
  auto SetFloatingTagIds(const std::vector<int> floating_tags) -> void;

private:
  auto ProcessObservations() -> void;
  auto ProcessTagObservations() -> void;
  auto ProcessCornersObservations() -> void;
  auto AddOdomForObservation(wpi::units::second_t timestamp) -> void;

  const wpi::fields::Field field_;
  gtsam::IncrementalFixedLagSmoother smoother_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values values_;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps_;
  wpi::math::TimeInterpolatableBuffer<Pose2d> interpolator_;
  std::queue<TagObservation> tag_observations_;
  std::queue<CornersObservation> corners_observations_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;

  std::shared_ptr<gtsam::PlanarGyroParams> imu_params_;

  std::vector<int> floating_tags_;
  std::set<int> seen_floating_tags_;

  Pose2d estimated_pose_;

  Pose2d previous_odom_;
  gtsam::Key previous_pose_key_;
  gtsam::Key previous_bias_key_;
  bool initialised_ = false;
};
}  // namespace gtsam_apriltag
