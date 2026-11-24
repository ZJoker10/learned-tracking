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

#ifndef TRAJECTORY_CONTROL__QUINTIC_POLYNOMIAL_PROFILE_HPP_
#define TRAJECTORY_CONTROL__QUINTIC_POLYNOMIAL_PROFILE_HPP_

#include "trajectory_control/trajectory_profile.hpp"

namespace trajectory_control
{

class QuinticPolynomialProfile : public TrajectoryProfile {
public:
  /// Construct a quintic polynomial trajectory.
  /// q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
  /// @param q0   Initial position
  /// @param qf   Final position
  /// @param v0   Initial velocity
  /// @param vf   Final velocity
  /// @param acc0 Initial acceleration
  /// @param accf Final acceleration
  /// @param T    Duration
  QuinticPolynomialProfile(
    double q0, double qf,
    double v0, double vf,
    double acc0, double accf,
    double T);

  TrajectoryState get_state(double t) const noexcept override;
  double duration() const noexcept override;

private:
  double a0_, a1_, a2_, a3_, a4_, a5_;
  double T_;
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__QUINTIC_POLYNOMIAL_PROFILE_HPP_
