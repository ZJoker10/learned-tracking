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

#ifndef TRAJECTORY_CONTROL__TRAJECTORY_UTILS_HPP_
#define TRAJECTORY_CONTROL__TRAJECTORY_UTILS_HPP_

#include <cmath>

namespace trajectory_control
{

/// Small value used to guard against division by (near-)zero and
/// to handle floating-point boundary comparisons.
inline constexpr double kEpsilon = 1e-12;

/// Minimum duration to prevent division by (near-)zero in coefficient formulas.
inline constexpr double kMinDuration = 1e-9;

/// Sanitize a time value for use in trajectory get_state() queries.
///
/// Returns 0.0 if t is NaN or infinity, otherwise returns t unchanged.
/// This prevents NaN/Inf propagation through trajectory computations.
inline double sanitize_time(double t) noexcept
{
  if (std::isnan(t) || !std::isfinite(t)) {
    return 0.0;
  }
  return t;
}

}  // namespace trajectory_control

#endif  // TRAJECTORY_CONTROL__TRAJECTORY_UTILS_HPP_
