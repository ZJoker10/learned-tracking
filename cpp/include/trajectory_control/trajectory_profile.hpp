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

#ifndef TRAJECTORY_CONTROL__TRAJECTORY_PROFILE_HPP_
#define TRAJECTORY_CONTROL__TRAJECTORY_PROFILE_HPP_

#include "trajectory_control/trajectory_state.hpp"

namespace trajectory_control
{

class TrajectoryProfile {
public:
  virtual ~TrajectoryProfile() = default;

  /// Return the kinematic state at time t (0 <= t <= duration())
  virtual TrajectoryState get_state(double t) const noexcept = 0;

  /// Total duration of this trajectory segment
  virtual double duration() const noexcept = 0;
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TRAJECTORY_PROFILE_HPP_
