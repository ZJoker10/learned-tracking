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

#include <algorithm>
#include <cmath>

#include "trajectory_control/s_curve_profile.hpp"
#include "trajectory_control/trajectory_utils.hpp"

namespace trajectory_control
{

SCurveProfile::SCurveProfile(
  double q0, double qf,
  double v_max, double a_max, double j_max)
: q0_(q0), qf_(qf),
  v_max_(std::abs(v_max)),
  a_max_(std::abs(a_max)),
  j_max_(std::abs(j_max)),
  dir_sign_(1.0),
  v_actual_(0.0), a_actual_(0.0),
  j_s_(0.0), a_s_(0.0), v_s_(0.0),
  t_j_(0.0), t_v1_(0.0), t_cruise_(0.0),
  t1_(0.0), t2_(0.0), t3_(0.0), t4_(0.0), t5_(0.0), t6_(0.0),
  T_(0.0),
  p1_(0.0), v1_(0.0),
  p2_(0.0), v2_(0.0),
  p3_(0.0),
  p4_(0.0),
  p5_(0.0), v5_(0.0),
  p6_(0.0), v6_(0.0)
{
  // Guard against NaN/Inf inputs — fall back to zero motion
  if (std::isnan(q0) || std::isnan(qf) ||
    std::isnan(v_max) || std::isnan(a_max) || std::isnan(j_max) ||
    !std::isfinite(q0) || !std::isfinite(qf) ||
    !std::isfinite(v_max) || !std::isfinite(a_max) || !std::isfinite(j_max))
  {
    q0_ = std::isfinite(q0) ? q0 : 0.0;
    qf_ = q0_;
    return;
  }

  // Direction of motion
  double s = qf_ - q0_;
  dir_sign_ = (s >= 0.0) ? 1.0 : -1.0;
  s = std::abs(s);

  // Guard against degenerate inputs: zero or near-zero distance / velocity /
  // acceleration / jerk. Any of these would cause degenerate trajectories.
  if (s < kEpsilon || v_max_ < kEpsilon || a_max_ < kEpsilon || j_max_ < kEpsilon) {
    // No motion — zero-duration profile at initial position
    qf_ = q0_;
    return;
  }

  // --- Compute profile parameters ---

  // Velocity at the boundary between the "reaches a_max" and "triangular"
  // acceleration regimes: v_ramp = a_max² / j_max
  double v_ramp = a_max_ * a_max_ / j_max_;

  // Distance at which a_max is just reached (at v = v_ramp):
  //   s_boundary = 2 * a_max³ / j_max²
  double s_boundary = 2.0 * a_max_ * a_max_ * a_max_ / (j_max_ * j_max_);

  // Determine whether we can reach a_max given the constraints
  bool can_reach_amax = (v_max_ >= v_ramp) && (s >= s_boundary);

  // --- Compute achievable cruise velocity v_actual ---
  //
  // Two regimes depending on whether we reach a_max:
  //
  // Regime A (reach a_max):  s = v²/a_max + v*a_max/j_max
  //   v_ach = ( -a_max²/j_max + sqrt((a_max²/j_max)² + 4*s*a_max) ) / 2
  //
  // Regime B (triangular):   s = 2 * v^(3/2) / sqrt(j_max)
  //   v_ach = ( s * sqrt(j_max) / 2 )^(2/3)

  if (can_reach_amax) {
    // Regime A: solve quadratic for v
    double v_ach = (-v_ramp + std::sqrt(v_ramp * v_ramp + 4.0 * s * a_max_)) / 2.0;
    v_actual_ = std::min(v_ach, v_max_);
    a_actual_ = a_max_;
    t_j_ = a_actual_ / j_max_;
    t_v1_ = v_actual_ / a_actual_ - t_j_;
  } else {
    // Regime B: triangular acceleration profile
    double v_ach = std::pow(s * std::sqrt(j_max_) / 2.0, 2.0 / 3.0);
    v_actual_ = std::min(v_ach, v_max_);
    a_actual_ = std::sqrt(j_max_ * v_actual_);
    t_j_ = a_actual_ / j_max_;
    t_v1_ = 0.0;
  }

  // Guard against numerical issues making t_v1_ slightly negative
  if (t_v1_ < 0.0) {
    t_v1_ = 0.0;
  }

  // --- Compute acceleration distance and cruise time ---
  //
  // For a symmetric acceleration profile (phases 1-3):
  //   d_accel = v_actual * (t_j_ + t_v1_ / 2)
  //
  // This formula works in both regimes:
  //   - Regime A (t_v1_ >= 0): d_accel = v²/(2*a) + v*a/(2*j_max)
  //   - Regime B (t_v1_ = 0):  d_accel = v * sqrt(v/j_max)

  double d_accel = v_actual_ * (t_j_ + t_v1_ / 2.0);
  double s_remaining = s - 2.0 * d_accel;

  if (s_remaining <= 0.0) {
    // Due to clamping to v_max or numerical rounding, a short decel-only
    // remainder may appear. Clamp to zero.
    t_cruise_ = 0.0;
  } else {
    t_cruise_ = s_remaining / v_actual_;
  }

  // --- Compute phase switch times ---
  t1_ = t_j_;
  t2_ = t1_ + t_v1_;
  t3_ = t2_ + t_j_;
  t4_ = t3_ + t_cruise_;
  t5_ = t4_ + t_j_;
  t6_ = t5_ + t_v1_;
  T_ = t6_ + t_j_;

  // --- Precompute boundary states (signed, in direction of motion) ---
  j_s_ = j_max_ * dir_sign_;
  a_s_ = a_actual_ * dir_sign_;
  v_s_ = v_actual_ * dir_sign_;

  // Phase 1: 0 → t1
  //   j = +j_s_,  a = j_s_ * τ,  v = 0.5 * j_s_ * τ²,  p = q0 + (1/6) * j_s_ * τ³
  double t_j_sq = t_j_ * t_j_;
  double t_j_cu = t_j_sq * t_j_;
  v1_ = 0.5 * j_s_ * t_j_sq;
  p1_ = q0_ + (1.0 / 6.0) * j_s_ * t_j_cu;

  // Phase 2: t1 → t2
  //   j = 0,  a = a_s_,  v = v1 + a_s_ * τ,  p = p1 + v1*τ + 0.5*a_s_*τ²
  double t_v1_sq = t_v1_ * t_v1_;
  v2_ = v1_ + a_s_ * t_v1_;
  p2_ = p1_ + v1_ * t_v1_ + 0.5 * a_s_ * t_v1_sq;

  // Phase 3: t2 → t3
  //   j = -j_s_,  a = a_s_ - j_s_*τ,  v = v2 + a_s_*τ - 0.5*j_s_*τ²,
  //   p = p2 + v2*τ + 0.5*a_s_*τ² - (1/6)*j_s_*τ³
  p3_ = p2_ + v2_ * t_j_ + 0.5 * a_s_ * t_j_sq - (1.0 / 6.0) * j_s_ * t_j_cu;

  // Phase 4: t3 → t4
  //   j = 0, a = 0, v = v_s_, p = p3 + v_s_ * τ
  p4_ = p3_ + v_s_ * t_cruise_;

  // Phase 5: t4 → t5
  //   j = -j_s_,  a = -j_s_*τ,  v = v_s_ - 0.5*j_s_*τ²,
  //   p = p4 + v_s_*τ - (1/6)*j_s_*τ³
  v5_ = v_s_ - 0.5 * j_s_ * t_j_sq;
  p5_ = p4_ + v_s_ * t_j_ - (1.0 / 6.0) * j_s_ * t_j_cu;

  // Phase 6: t5 → t6
  //   j = 0,  a = -a_s_,  v = v5 + (-a_s_)*τ,  p = p5 + v5*τ + 0.5*(-a_s_)*τ²
  v6_ = v5_ - a_s_ * t_v1_;
  p6_ = p5_ + v5_ * t_v1_ - 0.5 * a_s_ * t_v1_sq;
}

TrajectoryState SCurveProfile::get_state(double t) const noexcept
{
  TrajectoryState state;
  t = sanitize_time(t);

  // Handle out-of-range times
  if (t <= 0.0) {
    state.position = q0_;
    state.velocity = 0.0;
    state.acceleration = 0.0;
    state.jerk = 0.0;
    state.time = 0.0;
    return state;
  }

  if (t >= T_) {
    state.position = qf_;
    state.velocity = 0.0;
    state.acceleration = 0.0;
    state.jerk = 0.0;
    state.time = T_;
    return state;
  }

  double tau;  // time within current phase

  if (t < t1_) {
    // Phase 1: Increasing acceleration
    //   j = +j_s_,  a = j_s_ * τ,  v = 0.5 * j_s_ * τ²,  p = q0 + (1/6)*j_s_*τ³
    tau = t;
    state.jerk = j_s_;
    state.acceleration = j_s_ * tau;
    state.velocity = 0.5 * j_s_ * tau * tau;
    state.position = q0_ + (1.0 / 6.0) * j_s_ * tau * tau * tau;
  } else if (t < t2_) {
    // Phase 2: Constant acceleration
    //   j = 0,  a = a_s_,  v = v1 + a_s_*τ,  p = p1 + v1*τ + 0.5*a_s_*τ²
    tau = t - t1_;
    state.jerk = 0.0;
    state.acceleration = a_s_;
    state.velocity = v1_ + a_s_ * tau;
    state.position = p1_ + v1_ * tau + 0.5 * a_s_ * tau * tau;
  } else if (t < t3_) {
    // Phase 3: Decreasing acceleration
    //   j = -j_s_,  a = a_s_ - j_s_*τ,  v = v2 + a_s_*τ - 0.5*j_s_*τ²,
    //   p = p2 + v2*τ + 0.5*a_s_*τ² - (1/6)*j_s_*τ³
    tau = t - t2_;
    state.jerk = -j_s_;
    state.acceleration = a_s_ - j_s_ * tau;
    state.velocity = v2_ + a_s_ * tau - 0.5 * j_s_ * tau * tau;
    state.position = p2_ + v2_ * tau + 0.5 * a_s_ * tau * tau -
      (1.0 / 6.0) * j_s_ * tau * tau * tau;
  } else if (t < t4_) {
    // Phase 4: Constant velocity
    //   j = 0,  a = 0,  v = v_s_,  p = p3 + v_s_*τ
    tau = t - t3_;
    state.jerk = 0.0;
    state.acceleration = 0.0;
    state.velocity = v_s_;
    state.position = p3_ + v_s_ * tau;
  } else if (t < t5_) {
    // Phase 5: Increasing deceleration
    //   j = -j_s_,  a = -j_s_*τ,  v = v_s_ - 0.5*j_s_*τ²,
    //   p = p4 + v_s_*τ - (1/6)*j_s_*τ³
    tau = t - t4_;
    state.jerk = -j_s_;
    state.acceleration = -j_s_ * tau;
    state.velocity = v_s_ - 0.5 * j_s_ * tau * tau;
    state.position = p4_ + v_s_ * tau - (1.0 / 6.0) * j_s_ * tau * tau * tau;
  } else if (t < t6_) {
    // Phase 6: Constant deceleration
    //   j = 0,  a = -a_s_,  v = v5 - a_s_*τ,  p = p5 + v5*τ - 0.5*a_s_*τ²
    tau = t - t5_;
    state.jerk = 0.0;
    state.acceleration = -a_s_;
    state.velocity = v5_ - a_s_ * tau;
    state.position = p5_ + v5_ * tau - 0.5 * a_s_ * tau * tau;
  } else {
    // Phase 7: Decreasing deceleration
    //   j = +j_s_,  a = -a_s_ + j_s_*τ,  v = v6 - a_s_*τ + 0.5*j_s_*τ²,
    //   p = p6 + v6*τ - 0.5*a_s_*τ² + (1/6)*j_s_*τ³
    tau = t - t6_;
    state.jerk = j_s_;
    state.acceleration = -a_s_ + j_s_ * tau;
    state.velocity = v6_ - a_s_ * tau + 0.5 * j_s_ * tau * tau;
    state.position = p6_ + v6_ * tau - 0.5 * a_s_ * tau * tau +
      (1.0 / 6.0) * j_s_ * tau * tau * tau;
  }

  state.time = t;
  return state;
}

double SCurveProfile::duration() const noexcept
{
  return T_;
}

}  // namespace trajectory_control
