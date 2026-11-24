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

#ifndef TRAJECTORY_CONTROL__TRAPEZOIDAL_PROFILE_HPP_
#define TRAJECTORY_CONTROL__TRAPEZOIDAL_PROFILE_HPP_

#include "trajectory_control/trajectory_profile.hpp"

namespace trajectory_control
{

class TrapezoidalProfile : public TrajectoryProfile {
public:
  /// Direction of motion along the trajectory.
  enum class Direction
  {
    POSITIVE = 1,   ///< Moving from lower to higher position (q0 < qf)
    NEGATIVE = -1   ///< Moving from higher to lower position (q0 > qf)
  };

  /// Construct a trapezoidal velocity profile.
  /// @param q0     Initial position
  /// @param qf     Final position
  /// @param v_max  Maximum absolute velocity
  /// @param a_max  Maximum absolute acceleration
  TrapezoidalProfile(double q0, double qf, double v_max, double a_max);

  TrajectoryState get_state(double t) const noexcept override;
  double duration() const noexcept override;

private:
  double q0_;
  double qf_;
  double a_max_;
  double t_blend_;
  double t_total_;
  double v_cruise_;
  Direction dir_sign_;
  const double signed_a_max_;  // a_max_ * direction sign; cached for hot-path get_state()
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TRAPEZOIDAL_PROFILE_HPP_
