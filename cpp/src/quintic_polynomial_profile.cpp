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

#include "trajectory_control/quintic_polynomial_profile.hpp"
#include "trajectory_control/trajectory_utils.hpp"

namespace trajectory_control
{

QuinticPolynomialProfile::QuinticPolynomialProfile(
  double q0, double qf,
  double v0, double vf,
  double acc0, double accf,
  double T)
: T_(T)
{
  // Guard against NaN/Inf inputs — fall back to zero-duration hold.
  if (std::isnan(q0) || std::isnan(qf) ||
    std::isnan(v0) || std::isnan(vf) ||
    std::isnan(acc0) || std::isnan(accf) ||
    !std::isfinite(q0) || !std::isfinite(qf) ||
    !std::isfinite(v0) || !std::isfinite(vf) ||
    !std::isfinite(acc0) || !std::isfinite(accf) ||
    std::isnan(T))
  {
    T_ = kMinDuration;
    a0_ = std::isfinite(q0) ? q0 : 0.0;
    a1_ = 0.0;
    a2_ = 0.0;
    a3_ = 0.0;
    a4_ = 0.0;
    a5_ = 0.0;
    return;
  }

  // Guard against zero or negative duration: clamp to minimum to prevent NaN.
  if (T_ <= 0.0) {
    T_ = kMinDuration;
  }
  // Solve for coefficients of q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
  //
  // Constraints:
  //   q(0)   = q0   -> a0 = q0
  //   q'(0)  = v0   -> a1 = v0
  //   q''(0) = acc0 -> a2 = acc0 / 2
  //   q(T)   = qf
  //   q'(T)  = vf
  //   q''(T) = accf
  //
  // Closed-form solution (verified):
  double T2 = T_ * T_;
  double T3 = T2 * T_;
  double T4 = T3 * T_;
  double T5 = T4 * T_;

  a0_ = q0;
  a1_ = v0;
  a2_ = acc0 / 2.0;

  a3_ = (20.0 * (qf - q0) - (8.0 * vf + 12.0 * v0) * T_ - (3.0 * acc0 - accf) * T2) / (2.0 * T3);
  a4_ = (30.0 * (q0 - qf) + (14.0 * vf + 16.0 * v0) * T_ + (3.0 * acc0 - 2.0 * accf) * T2) /
    (2.0 * T4);
  a5_ = (12.0 * (qf - q0) - (6.0 * vf + 6.0 * v0) * T_ - (acc0 - accf) * T2) / (2.0 * T5);
}

TrajectoryState QuinticPolynomialProfile::get_state(double t) const noexcept
{
  TrajectoryState state;

  t = sanitize_time(t);

  if (t < 0.0) {
    t = 0.0;
  } else if (t > T_) {
    t = T_;
  }

  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  state.position = a0_ + a1_ * t + a2_ * t2 + a3_ * t3 + a4_ * t4 + a5_ * t5;
  state.velocity = a1_ + 2.0 * a2_ * t + 3.0 * a3_ * t2 + 4.0 * a4_ * t3 + 5.0 * a5_ * t4;
  state.acceleration = 2.0 * a2_ + 6.0 * a3_ * t + 12.0 * a4_ * t2 + 20.0 * a5_ * t3;
  state.jerk = 6.0 * a3_ + 24.0 * a4_ * t + 60.0 * a5_ * t2;
  state.time = t;

  return state;
}

double QuinticPolynomialProfile::duration() const noexcept
{
  return T_;
}

}  // namespace trajectory_control
