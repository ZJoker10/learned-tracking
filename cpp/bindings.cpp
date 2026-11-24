// Python bindings for the trajectory_control planner core.
//
// The planners are the same C++ implementations shipped in the ROS 2 package
// `trajectory_control`; they are vendored here so the benchmark is installable
// with plain `pip install .` (no ROS toolchain required).

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <utility>
#include <vector>

#include "trajectory_control/cubic_polynomial_profile.hpp"
#include "trajectory_control/multi_segment_planner.hpp"
#include "trajectory_control/quintic_polynomial_profile.hpp"
#include "trajectory_control/s_curve_profile.hpp"
#include "trajectory_control/trapezoidal_profile.hpp"

namespace py = pybind11;
using trajectory_control::CubicPolynomialProfile;
using trajectory_control::MultiSegmentPlanner;
using trajectory_control::QuinticPolynomialProfile;
using trajectory_control::SCurveProfile;
using trajectory_control::TrajectoryProfile;
using trajectory_control::TrajectoryState;
using trajectory_control::TrapezoidalProfile;

namespace
{

/// Uniform handle so Python can treat every profile the same way.
class Planner
{
public:
  explicit Planner(std::shared_ptr<TrajectoryProfile> impl) : impl_(std::move(impl)) {}

  TrajectoryState state(double t) const noexcept { return impl_->get_state(t); }

  double duration() const noexcept { return impl_->duration(); }

  /// Sample the profile at a uniform grid. Returns a list of TrajectoryState.
  std::vector<TrajectoryState> sample(double t0, double t1, std::size_t n) const
  {
    std::vector<TrajectoryState> out;
    out.reserve(n);
    if (n == 0) {
      return out;
    }
    const double dt = (n == 1) ? 0.0 : (t1 - t0) / static_cast<double>(n - 1);
    for (std::size_t i = 0; i < n; ++i) {
      out.push_back(impl_->get_state(t0 + dt * static_cast<double>(i)));
    }
    return out;
  }

  std::shared_ptr<TrajectoryProfile> raw() const noexcept { return impl_; }

private:
  std::shared_ptr<TrajectoryProfile> impl_;
};

/// Multi-segment plan that Python can build incrementally.
class SegmentPlan
{
public:
  SegmentPlan() = default;

  void add(const Planner & p) { impl_.add_segment(p.raw()); }

  void reserve(std::size_t n) { impl_.reserve(n); }

  void reset() noexcept { impl_.reset(); }

  TrajectoryState state(double t) const noexcept { return impl_.get_state(t); }

  double duration() const noexcept { return impl_.duration(); }

  bool finished(double t) const noexcept { return impl_.has_finished(t); }

private:
  MultiSegmentPlanner impl_;
};

}  // namespace

PYBIND11_MODULE(_core, m)
{
  m.doc() = "Bounded-jerk / polynomial trajectory profiles (C++ core)";

  py::class_<TrajectoryState>(m, "TrajectoryState")
    .def_readonly("position", &TrajectoryState::position)
    .def_readonly("velocity", &TrajectoryState::velocity)
    .def_readonly("acceleration", &TrajectoryState::acceleration)
    .def_readonly("jerk", &TrajectoryState::jerk)
    .def_readonly("time", &TrajectoryState::time)
    .def("is_finite", &TrajectoryState::is_finite)
    .def("__repr__", [](const TrajectoryState & s) {
      return "TrajectoryState(pos=" + std::to_string(s.position) + ", vel=" +
             std::to_string(s.velocity) + ", acc=" + std::to_string(s.acceleration) +
             ", jerk=" + std::to_string(s.jerk) + ", t=" + std::to_string(s.time) + ")";
    });

  py::class_<Planner>(m, "Planner")
    .def("state", &Planner::state, py::arg("t"))
    .def("duration", &Planner::duration)
    .def("sample", &Planner::sample, py::arg("t0"), py::arg("t1"), py::arg("n"));

  py::class_<SegmentPlan>(m, "SegmentPlan")
    .def(py::init<>())
    .def("add", &SegmentPlan::add, py::arg("planner"))
    .def("reserve", &SegmentPlan::reserve, py::arg("n"))
    .def("reset", &SegmentPlan::reset)
    .def("state", &SegmentPlan::state, py::arg("t"))
    .def("duration", &SegmentPlan::duration)
    .def("finished", &SegmentPlan::finished, py::arg("t"));

  m.def(
    "trapezoidal", [](double q0, double qf, double v_max, double a_max) {
      return Planner(std::make_shared<TrapezoidalProfile>(q0, qf, v_max, a_max));
    },
    py::arg("q0"), py::arg("qf"), py::arg("v_max"), py::arg("a_max"));

  m.def(
    "cubic", [](double q0, double qf, double v0, double vf, double T) {
      return Planner(std::make_shared<CubicPolynomialProfile>(q0, qf, v0, vf, T));
    },
    py::arg("q0"), py::arg("qf"), py::arg("v0"), py::arg("vf"), py::arg("T"));

  m.def(
    "quintic",
    [](double q0, double qf, double v0, double vf, double a0, double af, double T) {
      return Planner(std::make_shared<QuinticPolynomialProfile>(q0, qf, v0, vf, a0, af, T));
    },
    py::arg("q0"), py::arg("qf"), py::arg("v0"), py::arg("vf"), py::arg("a0"), py::arg("af"),
    py::arg("T"));

  m.def(
    "s_curve",
    [](double q0, double qf, double v_max, double a_max, double j_max) {
      return Planner(std::make_shared<SCurveProfile>(q0, qf, v_max, a_max, j_max));
    },
    py::arg("q0"), py::arg("qf"), py::arg("v_max"), py::arg("a_max"), py::arg("j_max"));
}
