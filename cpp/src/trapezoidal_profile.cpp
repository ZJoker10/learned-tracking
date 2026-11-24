// Copyright (c) 2026 Zeyad Kandil
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cmath>

#include "trajectory_control/trapezoidal_profile.hpp"
#include "trajectory_control/trajectory_utils.hpp"

namespace trajectory_control
{

TrapezoidalProfile::TrapezoidalProfile(
  double q0, double qf,
  double v_max, double a_max)
: q0_(q0), qf_(qf), a_max_(std::abs(a_max)),
  t_blend_(0.0), t_total_(0.0), v_cruise_(0.0),
  dir_sign_((qf >= q0 && std::isfinite(qf) && std::isfinite(q0)) ?
    Direction::POSITIVE : Direction::NEGATIVE),
  signed_a_max_(a_max_ * static_cast<double>(dir_sign_))
{
  // Guard against NaN/Inf inputs — fall back to zero motion
  if (std::isnan(q0) || std::isnan(qf) ||
    std::isnan(v_max) || std::isnan(a_max) ||
    !std::isfinite(v_max) || !std::isfinite(a_max))
  {
    q0_ = std::isfinite(q0) ? q0 : 0.0;
    qf_ = q0_;
    return;
  }

  double distance = qf_ - q0_;
  // dir_sign_ already computed correctly in the initializer list
  distance = std::abs(distance);
  v_max = std::abs(v_max);

  // Guard against degenerate inputs: zero or near-zero distance / velocity / acceleration.
  // Any of these would cause division by zero or NaN downstream.
  if (distance < kEpsilon || a_max_ < kEpsilon || v_max < kEpsilon) {
    // No motion — treat as zero-duration profile at initial position
    qf_ = q0_;
    return;
  }

  if (distance >= (v_max * v_max) / a_max_) {
    // Full trapezoid: accelerate, cruise, decelerate
    t_blend_ = v_max / a_max_;
    t_total_ = (distance / v_max) + t_blend_;
    v_cruise_ = v_max * static_cast<double>(dir_sign_);
  } else {
    // Triangular profile: no cruise phase
    t_blend_ = std::sqrt(distance / a_max_);
    t_total_ = 2.0 * t_blend_;
    v_cruise_ = (a_max_ * t_blend_) * static_cast<double>(dir_sign_);
  }
}

TrajectoryState TrapezoidalProfile::get_state(double t) const noexcept
{
  TrajectoryState state;

  t = sanitize_time(t);

  // Zero-duration profile: hold at initial position
  if (t_total_ < kEpsilon) {
    state.position = q0_;
    state.velocity = 0.0;
    state.acceleration = 0.0;
    state.time = t;
    return state;
  }

  if (t <= 0.0) {
    state.position = q0_;
    state.velocity = 0.0;
    state.acceleration = 0.0;
    state.time = 0.0;
    return state;
  } else if (t <= t_blend_) {
    // --- Accelerate phase ---
    // Constant acceleration a_max from rest.
    //   v(t) = a_max * t
    //   q(t) = q0 + 0.5 * a_max * t^2
    state.acceleration = signed_a_max_;
    state.velocity = state.acceleration * t;
    state.position = q0_ + 0.5 * state.acceleration * t * t;
  } else if (t <= t_total_ - t_blend_ + kEpsilon) {
    // --- Cruise phase ---
    // Constant velocity v_cruise; position is the accelerate-phase endpoint
    // plus linear travel at cruise speed for the remaining time.
    //   pos_at_blend = q(t_blend) = q0 + 0.5 * a_max * t_blend^2
    //   q(t) = pos_at_blend + v_cruise * (t - t_blend)
    // kEpsilon prevents floating-point edge misses at the
    // transition from cruise to decelerate.
    state.acceleration = 0.0;
    state.velocity = v_cruise_;
    double pos_at_blend = q0_ + 0.5 * signed_a_max_ * t_blend_ * t_blend_;
    state.position = pos_at_blend + v_cruise_ * (t - t_blend_);
  } else if (t < t_total_) {
    // --- Decelerate phase ---
    // Constant deceleration -a_max back to rest. t_dec counts down from
    // t_total_ to 0, symmetric to the accelerate phase.
    //   v(t) = a_max * t_dec
    //   q(t) = qf - 0.5 * a_max * t_dec^2
    // where t_dec = t_total - t.
    double t_dec = t_total_ - t;
    state.acceleration = -signed_a_max_;
    state.velocity = signed_a_max_ * t_dec;
    state.position = qf_ - 0.5 * signed_a_max_ * t_dec * t_dec;
  } else {
    // After motion completes
    state.position = qf_;
    state.velocity = 0.0;
    state.acceleration = 0.0;
  }

  state.time = t;
  return state;
}

double TrapezoidalProfile::duration() const noexcept
{
  return t_total_;
}

}  // namespace trajectory_control
