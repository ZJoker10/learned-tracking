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

#include "trajectory_control/cubic_polynomial_profile.hpp"
#include "trajectory_control/trajectory_utils.hpp"

namespace trajectory_control
{

CubicPolynomialProfile::CubicPolynomialProfile(
  double q0, double qf,
  double v0, double vf,
  double T)
: T_(T)
{
  // Guard against NaN/Inf inputs — fall back to zero-duration hold.
  if (std::isnan(q0) || std::isnan(qf) ||
    std::isnan(v0) || std::isnan(vf) ||
    !std::isfinite(q0) || !std::isfinite(qf) ||
    !std::isfinite(v0) || !std::isfinite(vf) ||
    std::isnan(T))
  {
    T_ = kMinDuration;
    a0_ = std::isfinite(q0) ? q0 : 0.0;
    a1_ = 0.0;
    a2_ = 0.0;
    a3_ = 0.0;
    return;
  }

  // Guard against zero or negative duration: clamp to minimum to prevent NaN.
  if (T_ <= 0.0) {
    T_ = kMinDuration;
  }

  // Solve for coefficients:
  // q(0)   = q0  = a0
  // q'(0)  = v0  = a1
  // q(T)   = qf  = a0 + a1*T + a2*T^2 + a3*T^3
  // q'(T)  = vf  = a1 + 2*a2*T + 3*a3*T^2
  double T2 = T_ * T_;
  double T3 = T2 * T_;
  a0_ = q0;
  a1_ = v0;
  a2_ = (3.0 * (qf - q0) - (2.0 * v0 + vf) * T_) / T2;
  a3_ = (2.0 * (q0 - qf) + (v0 + vf) * T_) / T3;
}

TrajectoryState CubicPolynomialProfile::get_state(double t) const noexcept
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

  state.position = a0_ + a1_ * t + a2_ * t2 + a3_ * t3;
  state.velocity = a1_ + 2.0 * a2_ * t + 3.0 * a3_ * t2;
  state.acceleration = 2.0 * a2_ + 6.0 * a3_ * t;
  state.jerk = 6.0 * a3_;
  state.time = t;

  return state;
}

double CubicPolynomialProfile::duration() const noexcept
{
  return T_;
}

}  // namespace trajectory_control
