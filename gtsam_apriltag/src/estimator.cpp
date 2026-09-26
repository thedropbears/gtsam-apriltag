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
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

#include <algorithm>
#include <cmath>
#include <print>
#include <vector>

using namespace gtsam;
using namespace wpi::math;
using gtsam::symbol_shorthand::L;

namespace gtsam_apriltag
{
Estimator::Estimator(const wpi::fields::Field & field, wpi::units::second_t window_size)
: field_(field),
  interpolator_(wpi::math::TimeInterpolatableBuffer<Pose2d>(window_size)),
  odometry_noise_(gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{0.05, 0.05, 0.05}))
{
  ISAM2Params params;
  params.findUnusedFactorSlots = true;

  smoother_ = IncrementalFixedLagSmoother(window_size.value(), params);  // times are in s
}

auto Estimator::Reset() -> void
{
  // Make a new instance of the smoother
  smoother_ = IncrementalFixedLagSmoother(smoother_.smootherLag(), smoother_.params());
  initialised_ = false;
}

auto Estimator::AddObservation(
  const wpi::units::second_t timestamp, const int tag_id, const Transform3d & camera_to_tag,
  const Transform3d & base_to_camera, const wpi::units::meter_t uncertainty) -> void
{
  observations_.push(
    Observation{
      .timestamp = timestamp,
      .tag_id = tag_id,
      .camera_to_tag = camera_to_tag,
      .base_to_camera = base_to_camera,
      .uncertainty = uncertainty});
}

auto Estimator::Update(const Pose2d & odometry, const wpi::units::second_t timestamp) -> Pose2d
{
  const auto key = ToKey(timestamp);

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
  } else if (!smoother_.getISAM2().valueExists(key)) {
    // Calculate a between factor for our new odometry
    const auto delta = odometry - previous_odom_;
    graph_.add(BetweenFactor<Pose2>(previous_odom_key_, key, ToGtsamPose(delta), odometry_noise_));
    previous_odom_key_ = key;
    previous_odom_ = odometry;

    values_.insert(key, ToGtsamPose(GetPose() + delta));
  }
  timestamps_[key] = timestamp.value();

  // Process any observations now that we have the odom up to date
  interpolator_.AddSample(timestamp, odometry);
  ProcessObservations();

  smoother_.update(graph_, values_, timestamps_);
  for (size_t i = 0; i < 2; ++i) {  // Optionally perform multiple iSAM2 iterations
    smoother_.update();
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

auto Estimator::Print() const -> void
{
  smoother_.getFactors().print();
  for (const auto id : floating_tags_) {
    const auto pos = smoother_.calculateEstimate<gtsam::Point2>(L(id));
    std::println("Floating tag {}: {}", id, pos);
  }
}

auto Estimator::SetOdometryStdDevs(const double x, const double y, const double theta) -> void
{
  odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{x, y, theta});
}

auto Estimator::SetFloatingTagIds(const std::vector<int> floating_tags) -> void
{
  floating_tags_ = floating_tags;
}

auto Estimator::ProcessObservations() -> void
{
  // If we don't have at least one factor in the graph we skip adding
  if (!initialised_) {
    return;
  }

  const auto odom_history = interpolator_.GetInternalBuffer();

  while (!observations_.empty()) {
    const auto obs = observations_.front();
    observations_.pop();

    if (obs.timestamp < odom_history.front().first) {
      // If this observation is before our first odom, then throw it away
      continue;
    } else if (obs.timestamp > odom_history.back().first) {
      // This is after our most recent odometry
      continue;
    } else {
      // In the middle of our existing graph
      const auto base_to_tag = obs.base_to_camera + obs.camera_to_tag;  // TODO Check ordering
      const auto key = ToKey(obs.timestamp);

      if (std::ranges::contains(floating_tags_, obs.tag_id)) {
        const auto landmark_key = L(obs.tag_id);
        const auto trans = base_to_tag.Translation().ToTranslation2d();
        const auto range = trans.Norm().value();
        const auto bearing = gtsam::Rot2::atan2(trans.Y().value(), trans.X().value());

        // TODO Approximate bearing uncertainty at this range
        const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector2{obs.uncertainty.value(), obs.uncertainty.value()});
        const auto bearing_range_factor =
          BearingRangeFactor<Pose2, Point2>(key, landmark_key, bearing, range, noise);
        graph_.add(bearing_range_factor);
        if (
          !std::ranges::contains(values_.keys(), landmark_key) &&
          !smoother_.getISAM2().valueExists(landmark_key)) {
          values_.insert(landmark_key, Point2());
        }
        timestamps_[landmark_key] = obs.timestamp.value();
      } else {
        const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector2{obs.uncertainty.value(), obs.uncertainty.value()});
        const auto tag = field_.GetTagPose(obs.tag_id);
        if (!tag) {
          continue;
        }

        const auto landmark_factor = KnownLandmarkFactor<gtsam::Pose2>(
          key, gtsam::Point2(tag->X().value(), tag->Y().value()),
          gtsam::Point2(base_to_tag.X().value(), base_to_tag.Y().value()), noise);
        graph_.add(landmark_factor);
      }

      if (!std::ranges::contains(values_.keys(), key) && !smoother_.getISAM2().valueExists(key)) {
        const auto odom = interpolator_.Sample(obs.timestamp);
        const auto next_sample = std::upper_bound(
          odom_history.begin(), odom_history.end(), std::pair(obs.timestamp, Pose2d()),
          [](auto lhs, auto rhs) { return lhs.first < rhs.first; });
        const auto prev_sample = next_sample - 1;

        const auto delta = *odom - prev_sample->second;
        const auto prev_key = ToKey(prev_sample->first);
        graph_.add(BetweenFactor<Pose2>(prev_key, key, ToGtsamPose(delta), odometry_noise_));
        values_.insert(key, Pose2());
        timestamps_[key] = obs.timestamp.value();
      }
    }
  }
}
}  // namespace gtsam_apriltag
