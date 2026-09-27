#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "teleop_common.hpp"

namespace {

inline double clamp_scalar(
    double value,
    double lower,
    double upper) {

  return std::max(
      lower,
      std::min(
          value,
          upper));
}

inline bool finite_quaternion(
    const Eigen::Quaterniond& q) {

  return
      std::isfinite(q.w()) &&
      std::isfinite(q.x()) &&
      std::isfinite(q.y()) &&
      std::isfinite(q.z()) &&
      q.squaredNorm() >
          1e-10;
}

inline Eigen::Matrix3d create_frame_map() {

  Eigen::Matrix3d R;

  R <<
      kVirtuoseToFranka[0],
      kVirtuoseToFranka[1],
      kVirtuoseToFranka[2],

      kVirtuoseToFranka[3],
      kVirtuoseToFranka[4],
      kVirtuoseToFranka[5],

      kVirtuoseToFranka[6],
      kVirtuoseToFranka[7],
      kVirtuoseToFranka[8];

  return R;
}

double rate_limit(
    double target,
    double previous,
    double maximum_step) {

  return previous +

         clamp_scalar(
             target - previous,
             -maximum_step,
             maximum_step);
}

double norm3(
    const double value[3]) {

  return std::sqrt(

      value[0] * value[0] +
      value[1] * value[1] +
      value[2] * value[2]);
}

bool finite_vector3(
    const Eigen::Vector3d& v) {

  return

      std::isfinite(v.x()) &&
      std::isfinite(v.y()) &&
      std::isfinite(v.z());
}


Eigen::Vector3d clamp_components(
    const Eigen::Vector3d& value,
    double limit) {

  return Eigen::Vector3d(

      clamp_scalar(
          value.x(),
          -limit,
          limit),

      clamp_scalar(
          value.y(),
          -limit,
          limit),

      clamp_scalar(
          value.z(),
          -limit,
          limit));
}


// ============================================================
// RELATIVE ROTATION LIMIT
// ============================================================

Eigen::Matrix3d limit_relative_rotation(
    const Eigen::Matrix3d& rotation,
    double maximum_angle_rad,
    bool* limited) {

  if (limited) {

    *limited =
        false;
  }


  Eigen::AngleAxisd aa(
      rotation);


  if (!std::isfinite(
          aa.angle()) ||

      aa.angle() <
          1e-12) {

    return Eigen::Matrix3d::
        Identity();
  }


  double angle =
      aa.angle();


  if (angle >
      maximum_angle_rad) {

    angle =
        maximum_angle_rad;


    if (limited) {

      *limited =
          true;
    }
  }


  return Eigen::AngleAxisd(
             angle,
             aa.axis())
      .toRotationMatrix();
}


// ============================================================
// ANGULAR TARGET RATE LIMIT
// ============================================================

Eigen::Quaterniond step_quaternion_toward(
    const Eigen::Quaterniond& current_input,
    const Eigen::Quaterniond& target_input,
    double maximum_step_rad,
    bool* limited) {

  if (limited) {

    *limited =
        false;
  }


  Eigen::Quaterniond current =
      current_input.normalized();


  Eigen::Quaterniond target =
      target_input.normalized();


  if (current.coeffs().dot(
          target.coeffs()) <
      0.0) {

    target.coeffs() *=
        -1.0;
  }


  Eigen::Quaterniond relative =

      current.conjugate() *
      target;


  relative.normalize();


  Eigen::AngleAxisd aa(
      relative);


  if (!std::isfinite(
          aa.angle()) ||

      aa.angle() <
          1e-12) {

    return target;
  }


  double step_angle =
      aa.angle();


  if (step_angle >
      maximum_step_rad) {

    step_angle =
        maximum_step_rad;


    if (limited) {

      *limited =
          true;
    }
  }


  Eigen::Quaterniond result =

      current *

      Eigen::Quaterniond(

          Eigen::AngleAxisd(
              step_angle,
              aa.axis()));


  result.normalize();


  return result;
}

Eigen::Vector3d clamp_vector_norm(
    const Eigen::Vector3d& value,
    double maximum_norm) {

  const double magnitude =
      value.norm();


  if (magnitude <=
          maximum_norm ||

      magnitude <
          1e-12) {

    return value;
  }


  return

      value *

      (
          maximum_norm /
          magnitude);
}


// ------------------------------------------------------------
// World-frame rotation vector taking current -> target
// ------------------------------------------------------------

Eigen::Vector3d orientation_error_world(
    const Eigen::Quaterniond& current_input,
    const Eigen::Quaterniond& target_input) {

  const Eigen::Quaterniond current =
      current_input.normalized();


  const Eigen::Quaterniond target =
      target_input.normalized();


  const Eigen::Matrix3d relative_rotation =

      target.toRotationMatrix() *

      current.toRotationMatrix().transpose();


  Eigen::AngleAxisd aa(
      relative_rotation);


  if (!std::isfinite(
          aa.angle()) ||

      aa.angle() <
          1e-12) {

    return Eigen::Vector3d::
        Zero();
  }


  return

      aa.axis() *
      aa.angle();
}

}  // namespace
