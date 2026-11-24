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

#ifndef TRAJECTORY_CONTROL__TRAJECTORY_STATE_HPP_
#define TRAJECTORY_CONTROL__TRAJECTORY_STATE_HPP_

#include <cmath>
#include <ostream>

namespace trajectory_control
{

/// Kinematic state at a single point in time along a trajectory.
struct TrajectoryState
{
  /// Position [user units]
  double position = 0.0;
  /// Velocity [user units / s]
  double velocity = 0.0;
  /// Acceleration [user units / s^2]
  double acceleration = 0.0;
  /// Jerk (3rd derivative of position) [user units / s^3]
  double jerk = 0.0;
  /// Time at which this state is evaluated [s]
  double time = 0.0;

  /// Check whether all kinematic fields (position, velocity, acceleration, jerk) are finite.
  /// time is excluded because it is an independent variable.
  constexpr bool is_finite() const noexcept
  {
    return std::isfinite(position) && std::isfinite(velocity) &&
           std::isfinite(acceleration) && std::isfinite(jerk);
  }

  /// Equality comparison (exact, for testing).
  constexpr bool operator==(const TrajectoryState & other) const noexcept
  {
    return position == other.position && velocity == other.velocity &&
           acceleration == other.acceleration && jerk == other.jerk &&
           time == other.time;
  }

  /// Inequality comparison.
  constexpr bool operator!=(const TrajectoryState & other) const noexcept
  {
    return !(*this == other);
  }
};

/// Stream a TrajectoryState in the format:
///   [t=T pos=P vel=V acc=A jerk=J]
inline std::ostream & operator<<(std::ostream & os, const TrajectoryState & s)
{
  os << "[t=" << s.time
     << " pos=" << s.position
     << " vel=" << s.velocity
     << " acc=" << s.acceleration
     << " jerk=" << s.jerk
     << "]";
  return os;
}

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TRAJECTORY_STATE_HPP_
