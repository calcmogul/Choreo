// Copyright (c) TrajoptLib contributors

#include "trajopt/DifferentialTrajectoryGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <utility>
#include <vector>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/SolverExitCondition.hpp>

#include "trajopt/geometry/Rotation2.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/Cancellation.hpp"
#include "trajopt/util/TrajoptUtil.hpp"

// Physics notation in this file:
//
//   x = linear position
//   v = linear velocity
//   a = linear acceleration
//   θ = heading
//   ω = angular velocity
//   α = angular acceleration
//   F = force
//   τ = torque
//   t = time
//
// A `d` prefix to one of these means delta.

namespace trajopt {

inline Translation2d WheelToChassisSpeeds(double vl, double vr) {
  return Translation2d{(vl + vr) / 2, 0.0};
}

inline Translation2v WheelToChassisSpeeds(sleipnir::Variable vl,
                                          sleipnir::Variable vr) {
  return Translation2v{(vl + vr) / 2, 0.0};
}

DifferentialTrajectoryGenerator::DifferentialTrajectoryGenerator(
    DifferentialPathBuilder pathBuilder, int64_t handle)
    : path(pathBuilder.GetPath()), Ns(pathBuilder.GetControlIntervalCounts()) {
  namespace slp = sleipnir;

  // See equations just before (12.35) and (12.36) in
  // https://controls-in-frc.link/ for wheel acceleration equations.
  //
  //   dx/dt = v cosθ
  //   dy/dt = v sinθ
  //   dθ/dt = ω
  //   dvₗ/dt = (1/m + r_b²/J) Fₗ + (1/m - r_b²/J) Fᵣ
  //   dvᵣ/dt = (1/m - r_b²/J) Fₗ + (1/m + r_b²/J) Fᵣ
  //
  // where
  //
  //   v = (vₗ + vᵣ) / 2
  //   ω = (vᵣ - vₗ) / (2r_b)

  auto f = [this](const slp::VariableMatrix& x,
                  const slp::VariableMatrix& u) -> slp::VariableMatrix {
    slp::VariableMatrix xdot{5};

    const auto& m = path.drivetrain.mass;
    double r_b = path.drivetrain.trackwidth / 2;
    const auto& J = path.drivetrain.moi;

    Eigen::Matrix<double, 2, 2> B{
        {1.0 / m + r_b * r_b / J, 1.0 / m - r_b * r_b / J},
        {1.0 / m - r_b * r_b / J, 1.0 / m + r_b * r_b / J}};

    auto v = (x(3) + x(4)) / 2.0;
    xdot(0) = v * cos(x(2));  // NOLINT
    xdot(1) = v * sin(x(2));  // NOLINT
    xdot(2) = (x(4) - x(3)) / path.drivetrain.trackwidth;
    xdot.Segment(3, 2) = B * u;

    return xdot;
  };

  auto initialGuess = pathBuilder.CalculateInitialGuess();

  problem.Callback(
      [this, handle = handle](const sleipnir::SolverIterationInfo&) -> bool {
        constexpr int fps = 60;
        constexpr std::chrono::duration<double> timePerFrame{1.0 / fps};

        // FPS limit on sending updates
        static auto lastFrameTime = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (now - lastFrameTime < timePerFrame) {
          return trajopt::GetCancellationFlag();
        }

        lastFrameTime = now;

        auto soln = ConstructDifferentialSolution();
        for (auto& callback : this->path.callbacks) {
          callback(soln, handle);
        }

        return trajopt::GetCancellationFlag();
      });

  size_t wptCnt = path.waypoints.size();
  size_t sgmtCnt = path.waypoints.size() - 1;
  size_t sampTot = GetIndex(Ns, wptCnt - 1, 0) + 1;

  x.reserve(sampTot);
  y.reserve(sampTot);
  θ.reserve(sampTot);
  vl.reserve(sampTot);
  vr.reserve(sampTot);
  al.reserve(sampTot);
  ar.reserve(sampTot);

  Fl.reserve(sampTot);
  Fr.reserve(sampTot);

  dts.reserve(sgmtCnt);

  for (size_t index = 0; index < sampTot; ++index) {
    x.emplace_back(problem.DecisionVariable());
    y.emplace_back(problem.DecisionVariable());
    θ.emplace_back(problem.DecisionVariable());
    vl.emplace_back(problem.DecisionVariable());
    vr.emplace_back(problem.DecisionVariable());
    al.emplace_back(problem.DecisionVariable());
    ar.emplace_back(problem.DecisionVariable());

    Fl.emplace_back(problem.DecisionVariable());
    Fr.emplace_back(problem.DecisionVariable());
  }

  for (size_t sgmtIndex = 0; sgmtIndex < sgmtCnt; ++sgmtIndex) {
    dts.emplace_back(problem.DecisionVariable());
  }

  // Minimize total time
  sleipnir::Variable T_tot = 0;
  const double maxForce =
      path.drivetrain.wheelMaxTorque * 2 / path.drivetrain.wheelRadius;
  const auto maxAccel = maxForce / path.drivetrain.mass;
  const double maxDrivetrainVelocity =
      path.drivetrain.wheelRadius * path.drivetrain.wheelMaxAngularVelocity;
  const auto maxAngVel = maxDrivetrainVelocity * 2 / path.drivetrain.trackwidth;
  const auto maxAngAccel = maxAccel * 2 / path.drivetrain.trackwidth;
  for (size_t sgmtIndex = 0; sgmtIndex < Ns.size(); ++sgmtIndex) {
    auto& dt = dts.at(sgmtIndex);
    auto N_sgmt = Ns.at(sgmtIndex);
    auto T_sgmt = dt * static_cast<int>(N_sgmt);
    T_tot += T_sgmt;

    problem.SubjectTo(dt >= 0);
    problem.SubjectTo(dt * path.drivetrain.wheelRadius *
                          path.drivetrain.wheelMaxAngularVelocity <=
                      path.drivetrain.trackwidth);
    if (N_sgmt == 0) {
      dt.SetValue(0);
    } else {
      // Use initialGuess and Ns to find the dx, dy, dθ between wpts
      const auto sgmt_start = GetIndex(Ns, sgmtIndex);
      const auto sgmt_end = GetIndex(Ns, sgmtIndex + 1);
      const auto dx =
          initialGuess.x.at(sgmt_end) - initialGuess.x.at(sgmt_start);
      const auto dy =
          initialGuess.y.at(sgmt_end) - initialGuess.y.at(sgmt_start);
      const auto dist = std::hypot(dx, dy);
      const auto θ_0 = initialGuess.heading.at(sgmt_start);
      const auto θ_1 = initialGuess.heading.at(sgmt_end);
      const auto dθ = std::abs(AngleModulus(θ_1 - θ_0));

      auto maxLinearVel = maxDrivetrainVelocity;

      const auto angularTime =
          CalculateTrapezoidalTime(dθ, maxAngVel, maxAngAccel);
      maxLinearVel = std::min(maxLinearVel, dist / angularTime);

      const auto linearTime =
          CalculateTrapezoidalTime(dist, maxLinearVel, maxAccel);
      const double sgmtTime = angularTime + linearTime;

      dt.SetValue(sgmtTime / N_sgmt);
    }
  }
  problem.Minimize(std::move(T_tot));

  // Apply dynamics constraints
  for (size_t wptIndex = 0; wptIndex < wptCnt - 1; ++wptIndex) {
    size_t N_sgmt = Ns.at(wptIndex);
    auto dt = dts.at(wptIndex);

    for (size_t sampleIndex = 0; sampleIndex < N_sgmt; ++sampleIndex) {
      size_t index = GetIndex(Ns, wptIndex, sampleIndex);

      slp::VariableMatrix x_k{{x.at(index)},
                              {y.at(index)},
                              {θ.at(index)},
                              {vl.at(index)},
                              {vr.at(index)}};
      slp::VariableMatrix u_k{{Fl.at(index)}, {Fr.at(index)}};

      slp::VariableMatrix x_k_1{{x.at(index + 1)},
                                {y.at(index + 1)},
                                {θ.at(index + 1)},
                                {vl.at(index + 1)},
                                {vr.at(index + 1)}};
      slp::VariableMatrix u_k_1{{Fl.at(index + 1)}, {Fr.at(index + 1)}};

      // Dynamics constraints - direct collocation
      // (https://mec560sbu.github.io/2016/09/30/direct_collocation/)
      auto xdot_k = f(x_k, u_k);
      auto xdot_k_1 = f(x_k_1, u_k_1);
      auto xdot_c = -3 / (2 * dt) * (x_k - x_k_1) - 0.25 * (xdot_k + xdot_k_1);

      auto x_c = 0.5 * (x_k + x_k_1) + dt / 8 * (xdot_k - xdot_k_1);
      auto u_c = 0.5 * (u_k + u_k_1);

      problem.SubjectTo(xdot_c == f(x_c, u_c));

      problem.SubjectTo(al.at(index) == xdot_k(3));
      problem.SubjectTo(ar.at(index) == xdot_k(4));
    }
  }

  // Apply wheel power constraints
  for (size_t index = 0; index < sampTot; ++index) {
    double maxWheelVelocity =
        path.drivetrain.wheelRadius * path.drivetrain.wheelMaxAngularVelocity;

    // −vₘₐₓ < vₗ < vₘₐₓ
    problem.SubjectTo(-maxWheelVelocity < vl.at(index));
    problem.SubjectTo(vl.at(index) < maxWheelVelocity);

    // −vₘₐₓ < vᵣ < vₘₐₓ
    problem.SubjectTo(-maxWheelVelocity < vr.at(index));
    problem.SubjectTo(vr.at(index) < maxWheelVelocity);

    // τ = r x F
    // F = τ/r
    double maxWheelForce =
        path.drivetrain.wheelMaxTorque / path.drivetrain.wheelRadius;

    // friction = μmg
    double maxFrictionForce =
        path.drivetrain.wheelCoF * path.drivetrain.mass * 9.8;

    double maxForce = std::min(maxWheelForce, maxFrictionForce);

    // −Fₘₐₓ < Fₗ < Fₘₐₓ
    problem.SubjectTo(-maxForce < Fl.at(index));
    problem.SubjectTo(Fl.at(index) < maxForce);

    // −Fₘₐₓ < Fᵣ < Fₘₐₓ
    problem.SubjectTo(-maxForce < Fr.at(index));
    problem.SubjectTo(Fr.at(index) < maxForce);
  }

  for (size_t wptIndex = 0; wptIndex < wptCnt; ++wptIndex) {
    // First index of next wpt - 1
    size_t index = GetIndex(Ns, wptIndex, 0);

    Pose2v pose_k{x.at(index), y.at(index), {θ.at(index)}};
    Translation2v v_k = WheelToChassisSpeeds(vl.at(index), vr.at(index));
    auto ω_k = (vr.at(index) - vl.at(index)) / path.drivetrain.trackwidth;
    Translation2v a_k = WheelToChassisSpeeds(al.at(index), ar.at(index));
    auto α_k = (ar.at(index) - al.at(index)) / path.drivetrain.trackwidth;

    for (auto& constraint : path.waypoints.at(wptIndex).waypointConstraints) {
      std::visit(
          [&](auto&& arg) { arg.Apply(problem, pose_k, v_k, ω_k, a_k, α_k); },
          constraint);
    }
  }

  for (size_t sgmtIndex = 0; sgmtIndex < sgmtCnt; ++sgmtIndex) {
    size_t startIndex = GetIndex(Ns, sgmtIndex, 0);
    size_t endIndex = GetIndex(Ns, sgmtIndex + 1, 0);

    for (size_t index = startIndex; index < endIndex; ++index) {
      Pose2v pose_k{x.at(index), y.at(index), {θ.at(index)}};
      Translation2v v_k = WheelToChassisSpeeds(vl.at(index), vr.at(index));
      auto ω_k = (vr.at(index) - vl.at(index)) / path.drivetrain.trackwidth;
      Translation2v a_k = WheelToChassisSpeeds(al.at(index), ar.at(index));
      auto α_k = (ar.at(index) - al.at(index)) / path.drivetrain.trackwidth;

      for (auto& constraint :
           path.waypoints.at(sgmtIndex + 1).segmentConstraints) {
        std::visit(
            [&](auto&& arg) { arg.Apply(problem, pose_k, v_k, ω_k, a_k, α_k); },
            constraint);
      }
    }
  }

  ApplyInitialGuess(initialGuess);
}

expected<DifferentialSolution, sleipnir::SolverExitCondition>
DifferentialTrajectoryGenerator::Generate(bool diagnostics) {
  GetCancellationFlag() = 0;

  // tolerance of 1e-4 is 0.1 mm
  auto status = problem.Solve({.tolerance = 1e-4, .diagnostics = diagnostics});

  if (static_cast<int>(status.exitCondition) < 0 ||
      status.exitCondition ==
          sleipnir::SolverExitCondition::kCallbackRequestedStop) {
    return unexpected{status.exitCondition};
  } else {
    return ConstructDifferentialSolution();
  }
}

void DifferentialTrajectoryGenerator::ApplyInitialGuess(
    const DifferentialSolution& solution) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 0; sampleIndex < sampleTotal; ++sampleIndex) {
    x[sampleIndex].SetValue(solution.x[sampleIndex]);
    y[sampleIndex].SetValue(solution.y[sampleIndex]);
    θ[sampleIndex].SetValue(solution.heading[sampleIndex]);
  }

  vl[0].SetValue(0.0);
  vr[0].SetValue(0.0);
  al[0].SetValue(0.0);
  ar[0].SetValue(0.0);

  for (size_t sampleIndex = 1; sampleIndex < sampleTotal; ++sampleIndex) {
    double linearVelocity =
        std::hypot(solution.x[sampleIndex] - solution.x[sampleIndex - 1],
                   solution.y[sampleIndex] - solution.y[sampleIndex - 1]) /
        solution.dt[sampleIndex];
    double heading = solution.heading[sampleIndex];
    double last_heading = solution.heading[sampleIndex - 1];

    double ω =
        Rotation2d{heading}.RotateBy(-Rotation2d{last_heading}).Radians() /
        solution.dt[sampleIndex];
    vl[sampleIndex].SetValue(
        (linearVelocity - path.drivetrain.trackwidth / 2 * ω));
    vr[sampleIndex].SetValue(
        (linearVelocity + path.drivetrain.trackwidth / 2 * ω));
    al[sampleIndex].SetValue(
        (vl[sampleIndex].Value() - vl[sampleIndex - 1].Value()) /
        solution.dt[sampleIndex]);
    ar[sampleIndex].SetValue(
        (vr[sampleIndex].Value() - vr[sampleIndex - 1].Value()) /
        solution.dt[sampleIndex]);
  }
}

DifferentialSolution
DifferentialTrajectoryGenerator::ConstructDifferentialSolution() {
  std::vector<double> dtPerSample;
  for (size_t sgmtIndex = 0; sgmtIndex < Ns.size(); ++sgmtIndex) {
    auto N = Ns.at(sgmtIndex);
    auto dt = dts.at(sgmtIndex);

    double dt_value = dt.Value();
    for (size_t i = 0; i < N; ++i) {
      dtPerSample.push_back(dt_value);
    }
  }

  auto getValue = [](auto& var) { return var.Value(); };

  // TODO: Use std::ranges::to() from C++23
  auto vectorValue = [&](std::vector<sleipnir::Variable>& row) {
    auto view = row | std::views::transform(getValue);
    return std::vector<double>{std::begin(view), std::end(view)};
  };
  const auto& trackwidth = path.drivetrain.trackwidth;
  std::vector<double> ω;
  for (size_t sample = 0; sample < vl.size(); ++sample) {
    ω.push_back((vr.at(sample).Value() - vl.at(sample).Value()) / trackwidth);
  }
  return DifferentialSolution{
      dtPerSample,
      vectorValue(x),
      vectorValue(y),
      vectorValue(θ),
      vectorValue(vl),
      vectorValue(vr),
      ω,
      vectorValue(al),
      vectorValue(ar),
      vectorValue(Fl),
      vectorValue(Fr),
  };
}

}  // namespace trajopt
