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

#ifndef TRAJECTORY_CONTROL__MULTI_SEGMENT_PLANNER_HPP_
#define TRAJECTORY_CONTROL__MULTI_SEGMENT_PLANNER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

#include "trajectory_control/trajectory_profile.hpp"

namespace trajectory_control
{

class MultiSegmentPlanner {
public:
  MultiSegmentPlanner();

  /// Add a trajectory segment to the plan.
  void add_segment(std::shared_ptr<TrajectoryProfile> profile);

  /// Get the kinematic state at global time t.
  TrajectoryState get_state(double t) const noexcept;

  /// Total duration of all segments combined.
  double duration() const noexcept;

  /// Check whether the trajectory has completed by global time t.
  /// @param t  Global time (wall-clock seconds since trajectory start)
  /// @returns  true when t >= total duration of all segments combined
  bool has_finished(double t) const noexcept;

  /// Pre-allocate memory for the given number of segments.
  /// Call before adding many segments to avoid repeated reallocation.
  void reserve(std::size_t n);

  /// Reset the segment index back to the first segment for re-use.
  /// Call this before replaying the trajectory (e.g. in a loop).
  void reset() noexcept;

private:
  struct Segment
  {
    std::shared_ptr<TrajectoryProfile> profile;
    double start_time;  // global time when this segment begins
  };

  std::vector<Segment> segments_;
  mutable std::atomic<std::size_t> current_idx_;
};

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__MULTI_SEGMENT_PLANNER_HPP_
