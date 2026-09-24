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

#include "estimator.hpp"

#include "conversion.hpp"

using namespace gtsam;
using namespace wpi::math;

using symbol_shorthand::X;

namespace gtsam_apriltag
{
Estimator::Estimator(const wpi::fields::Field & field) : field_(field)
{
  ISAM2Params params;
  params.findUnusedFactorSlots = true;

  smoother_ = IncrementalFixedLagSmoother(5.0, params);  // times are in s
}

auto Estimator::Reset() -> void
{
  // Make a new instance of the smoother
  smoother_ = IncrementalFixedLagSmoother(smoother_.smootherLag(), smoother_.params());
  initialised_ = false;
}

auto Estimator::AddObservation(
  const double timestamp_seconds, const int tag_id, const Transform3d & camera_to_tag,
  const Transform3d & base_to_camera) -> void
{
  // If we don't have at least one factor in the graph we skip adding
  if (!initialised_) {
    return;
  }

  // If this observation is before our first odom, then throw it away

  // In the middle of our existing graph
  
  // This is after our most recent odometry
}

auto Estimator::Update(const Pose2d & odometry, const double timestamp_seconds) -> Pose2d
{
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{0.05, 0.05, 0.05});
  const auto key = X(static_cast<uint64_t>(timestamp_seconds * 1e6));

  // If this is the first update, then we can't add between factors
  if (!initialised_) {
    const auto gtsam_pose = ToGtsamPose(odometry);
    // Big uncertainty for first observation - then correct with tags
    gtsam::Vector3 sigmas{30.0, 30.0, 1.5};

    graph_.addPrior(key, gtsam_pose, gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    previous_odom_key_ = key;
    values_.insert(key, gtsam_pose);

    previous_odom_ = odometry;
    estimated_pose_ = odometry;
  } else {
    // Calculate a between factor for our new odometry
    const auto delta = odometry - previous_odom_;
    graph_.add(BetweenFactor<Pose2>(previous_odom_key_, key, ToGtsamPose(delta), noise));

    values_.insert(key, ToGtsamPose(GetPose() + delta));
  }
  timestamps_[key] = timestamp_seconds;
  try {
    smoother_.update(graph_, values_, timestamps_);
  } catch (...) {
    // Tried to insert the same value twice
  }

  timestamps_.clear();
  graph_.resize(0);
  values_.clear();

  if (initialised_) {
    estimated_pose_ = ToWpiPose(smoother_.calculateEstimate<gtsam::Pose2>(key));
  }

  initialised_ = true;

  return GetPose();
}

auto Estimator::GetPose() const -> Pose2d
{
  return estimated_pose_;
}
}  // namespace gtsam_apriltag
