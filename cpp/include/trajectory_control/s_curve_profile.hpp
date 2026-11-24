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

#ifndef TRAJECTORY_CONTROL__S_CURVE_PROFILE_HPP_
#define TRAJECTORY_CONTROL__S_CURVE_PROFILE_HPP_

#include "trajectory_control/trajectory_profile.hpp"

namespace trajectory_control
{

/// Bounded-jerk S-curve trajectory profile with 7 phases.
///
/// Generates a smooth trajectory from q0 to qf with bounded jerk,
/// acceleration, and velocity. The motion consists of 7 phases:
///
///   1. Increasing acceleration  (j = +j_max)
///   2. Constant acceleration    (j = 0)
///   3. Decreasing acceleration  (j = -j_max)
///   4. Constant velocity        (j = 0, a = 0)
///   5. Increasing deceleration  (j = -j_max)
///   6. Constant deceleration    (j = 0)
///   7. Decreasing deceleration  (j = +j_max)
///
/// Handles both trapezoidal-velocity (reach v_max) and triangular-velocity
/// (distance too short) cases, as well as cases where a_max is not reached.
class SCurveProfile : public TrajectoryProfile {
public:
  /// Construct a bounded-jerk S-curve trajectory.
  /// @param q0    Initial position
  /// @param qf    Final position
  /// @param v_max Maximum absolute velocity
  /// @param a_max Maximum absolute acceleration
  /// @param j_max Maximum absolute jerk
  SCurveProfile(double q0, double qf, double v_max, double a_max, double j_max);

  TrajectoryState get_state(double t) const noexcept override;
  double duration() const noexcept override;

private:
  double q0_;
  double qf_;
  double v_max_;
  double a_max_;
  double j_max_;

  // Direction of motion (+1 or -1)
  double dir_sign_;

  // Actual (achieved) peak values (always positive, direction applied in get_state)
  double v_actual_;
  double a_actual_;

  // Cached signed kinematic values (set once in constructor, read in hot-path get_state)
  double j_s_;  // j_max_ * dir_sign_
  double a_s_;  // a_actual_ * dir_sign_
  double v_s_;  // v_actual_ * dir_sign_

  // Phase durations
  double t_j_;       // jerk ramp time (phase 1, 3, 5, 7 duration)
  double t_v1_;      // constant acceleration time (phase 2, 6 duration)
  double t_cruise_;  // constant velocity time (phase 4 duration)

  // Phase switch times
  double t1_, t2_, t3_, t4_, t5_, t6_;

  // Total duration
  double T_;

  // Boundary states at phase transitions (signed, in direction of motion)
  double p1_, v1_;
  double p2_, v2_;
  double p3_;
  double p4_;
  double p5_, v5_;
  double p6_, v6_;
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__S_CURVE_PROFILE_HPP_
