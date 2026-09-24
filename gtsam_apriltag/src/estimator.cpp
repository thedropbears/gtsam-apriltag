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

#include <wpi/math/geometry/Pose3d.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

#include <algorithm>
#include <print>

using namespace gtsam;
using namespace wpi::math;

using symbol_shorthand::X;

namespace gtsam_apriltag
{
Estimator::Estimator(const wpi::fields::Field & field)
: field_(field),
  interpolator_(wpi::math::TimeInterpolatableBuffer<Pose2d>(wpi::units::second_t(5.0)))
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
  const wpi::units::second_t timestamp, const int tag_id, const Transform3d & camera_to_tag,
  const Transform3d & base_to_camera) -> void
{
  // If we don't have at least one factor in the graph we skip adding
  if (!initialised_) {
    return;
  }

  const auto base_to_tag = base_to_camera + camera_to_tag;  // TODO Check ordering
  const auto key = X(static_cast<uint64_t>(timestamp.value() * 1e6));

  const auto tag = field_.GetTagPose(tag_id);
  if (!tag) {
    return;
  }

  const auto odom_history = interpolator_.GetInternalBuffer();

  if (timestamp < odom_history.front().first) {
    // If this observation is before our first odom, then throw it away
    return;
  } else if (timestamp > odom_history.back().first) {
    // This is after our most recent odometry

  } else {
    // In the middle of our existing graph
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2{0.05, 0.05});
    const auto landmark_factor = KnownLandmarkFactor<gtsam::Pose2>(
      key, gtsam::Point2(tag->X().value(), tag->Y().value()),
      gtsam::Point2(base_to_tag.X().value(), base_to_tag.Y().value()), noise);
    graph_.add(landmark_factor);

    if (!std::ranges::contains(values_.keys(), key) && !smoother_.getISAM2().valueExists(key)) {
      const auto odom = interpolator_.Sample(timestamp);
      const auto next_sample = std::upper_bound(
        odom_history.begin(), odom_history.end(), std::pair(timestamp, Pose2d()),
        [](auto lhs, auto rhs) { return lhs.first < rhs.first; });
      const auto prev_sample = next_sample - 1;

      const auto delta = *odom - prev_sample->second;
      const auto prev_key = X(static_cast<uint64_t>(prev_sample->first.value() * 1e6));
      const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{0.05, 0.05, 0.05});
      graph_.add(BetweenFactor<Pose2>(prev_key, key, ToGtsamPose(delta), noise));
      values_.insert(key, Pose2());
      timestamps_[key] = timestamp.value();
    }
  }
}

auto Estimator::Update(const Pose2d & odometry, const wpi::units::second_t timestamp) -> Pose2d
{
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{0.05, 0.05, 0.05});
  const auto key = X(static_cast<uint64_t>(timestamp.value() * 1e6));

  if (smoother_.getISAM2().valueExists(key)) {
    return GetPose();
  }

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
    previous_odom_key_ = key;
    previous_odom_ = odometry;

    values_.insert(key, ToGtsamPose(GetPose() + delta));
  }
  timestamps_[key] = timestamp.value();

  interpolator_.AddSample(timestamp, odometry);
  // TODO add pending observations

  smoother_.update(graph_, values_, timestamps_);
  /*for (size_t i = 1; i < 2; ++i) {  // Optionally perform multiple iSAM2 iterations
    smoother_.update();
  }*/

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

auto Estimator::Print() const -> void
{
  smoother_.getFactors().print();
}
}  // namespace gtsam_apriltag
