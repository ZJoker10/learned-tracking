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

#include "trajectory_control/multi_segment_planner.hpp"
#include "trajectory_control/trajectory_utils.hpp"

namespace trajectory_control
{

MultiSegmentPlanner::MultiSegmentPlanner()
: current_idx_(0)
{
  // atomic current_idx_ is zero-initialized
}

void MultiSegmentPlanner::add_segment(std::shared_ptr<TrajectoryProfile> profile)
{
  if (!profile) {
    return;
  }
  double start = 0.0;
  if (!segments_.empty()) {
    const auto & last = segments_.back();
    start = last.start_time + last.profile->duration();
  }
  segments_.push_back({std::move(profile), start});
}

TrajectoryState MultiSegmentPlanner::get_state(double t) const noexcept
{
  t = sanitize_time(t);

  if (segments_.empty()) {
    TrajectoryState s;
    s.time = t;
    return s;
  }

  // Clamp time to valid range [0, total_duration]
  double total_dur = duration();
  if (t < 0.0) {
    t = 0.0;
  } else if (t > total_dur) {
    t = total_dur;
  }

  // Work with a local copy of the index for thread safety
  std::size_t idx = current_idx_.load();

  // Clamp index to valid range
  if (idx >= segments_.size()) {
    idx = segments_.size() - 1;
  }

  // Advance segment index forward if we've passed the next segment's start
  while (idx + 1 < segments_.size() &&
    t >= segments_[idx + 1].start_time)
  {
    ++idx;
  }

  // Go backward if we've gone before the current segment's start
  while (idx > 0 &&
    t < segments_[idx].start_time)
  {
    --idx;
  }

  // Store the updated index back
  current_idx_.store(idx);

  const Segment & seg = segments_[idx];
  double local_t = t - seg.start_time;

  // Clamp local time to segment bounds
  if (local_t < 0.0) {
    local_t = 0.0;
  }
  double seg_dur = seg.profile->duration();
  if (local_t > seg_dur) {
    local_t = seg_dur;
  }

  TrajectoryState state = seg.profile->get_state(local_t);
  state.time = t;
  return state;
}

double MultiSegmentPlanner::duration() const noexcept
{
  if (segments_.empty()) {
    return 0.0;
  }
  const Segment & last = segments_.back();
  return last.start_time + last.profile->duration();
}

bool MultiSegmentPlanner::has_finished(double t) const noexcept
{
  return t >= duration();
}

void MultiSegmentPlanner::reserve(std::size_t n)
{
  segments_.reserve(n);
}

void MultiSegmentPlanner::reset() noexcept
{
  current_idx_.store(0);
}

}  // namespace trajectory_control
