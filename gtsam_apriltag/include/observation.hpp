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

#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>

#include <array>

namespace gtsam_apriltag
{
struct CameraCalibration
{
  double fx;
  double fy;
  double s;
  double u0;
  double v0;
  double k1;
  double k2;
  double p1;
  double p2;

  CameraCalibration(
    double fx, double fy, double s, double u0, double v0, double k1, double k2, double p1 = 0.0,
    double p2 = 0.0)
  : fx(fx), fy(fy), s(s), u0(u0), v0(v0), k1(k1), k2(k2), p1(p1), p2(p2)
  {
  }
  CameraCalibration(std::array<double, 9> intrinsics, std::array<double, 8> distortion)
  : fx(intrinsics[0]),
    fy(intrinsics[4]),
    s(intrinsics[1]),
    u0(intrinsics[2]),
    v0(intrinsics[5]),
    k1(distortion[0]),
    k2(distortion[1]),
    p1(distortion[2]),
    p2(distortion[3])
  {
  }
};

struct TagObservation
{
  wpi::units::second_t timestamp;
  int tag_id;
  wpi::math::Transform3d camera_to_tag;
  wpi::math::Transform3d base_to_camera;
  wpi::units::radian_t bearing_uncertainty;
  wpi::units::meter_t range_uncertainty;

  TagObservation(
    wpi::units::second_t timestamp, int tag_id, wpi::math::Transform3d camera_to_tag,
    wpi::math::Transform3d base_to_camera,
    wpi::units::radian_t bearing_uncertainty = wpi::units::radian_t{0.05},
    wpi::units::meter_t range_uncertainty = wpi::units::meter_t{0.05})
  : timestamp(timestamp),
    tag_id(tag_id),
    camera_to_tag(camera_to_tag),
    base_to_camera(base_to_camera),
    bearing_uncertainty(bearing_uncertainty),
    range_uncertainty(range_uncertainty)
  {
  }
};

struct CornersObservation
{
  wpi::units::second_t timestamp;
  int tag_id;
  std::array<wpi::math::Translation2d, 4> corners;
  CameraCalibration camera_calibration;
  wpi::math::Transform3d base_to_camera;
  double pixel_uncertainty = 1.0;

  CornersObservation(
    wpi::units::second_t timestamp, int tag_id, std::array<wpi::math::Translation2d, 4> corners,
    CameraCalibration camera_calibration, wpi::math::Transform3d base_to_camera,
    double pixel_uncertainty = 1.0)
  : timestamp(timestamp),
    tag_id(tag_id),
    corners(corners),
    camera_calibration(camera_calibration),
    base_to_camera(base_to_camera),
    pixel_uncertainty(pixel_uncertainty)
  {
  }
};

}  // namespace gtsam_apriltag
