// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/geometry/HPolytope2.hpp"
#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Keep-in circle constraint.
 */
class TRAJOPT_DLLEXPORT KeepInCircleConstraint {
 public:
  /**
   * Constructs a KeepInCircleConstraint.
   *
   * @param center The circle's center.
   * @param radius The circle's radius.
   */
  KeepInCircleConstraint(Translation2d center, double radius)
      : m_center{std::move(center)}, m_radius{radius} {}

  /**
   * Applies this constraint to the given problem.
   *
   * @param problem The optimization problem.
   * @param pose The robot's pose.
   * @param robotRegion The 2D region the robot occupies.
   * @param linearVelocity The robot's linear velocity.
   * @param angularVelocity The robot's angular velocity.
   * @param linearAcceleration The robot's linear acceleration.
   * @param angularAcceleration The robot's angular acceleration.
   */
  void Apply(sleipnir::OptimizationProblem& problem, const Pose2v& pose,
             [[maybe_unused]] const HPolytope2v& robotRegion,
             [[maybe_unused]] const Translation2v& linearVelocity,
             [[maybe_unused]] const sleipnir::Variable& angularVelocity,
             [[maybe_unused]] const Translation2v& linearAcceleration,
             [[maybe_unused]] const sleipnir::Variable& angularAcceleration) {
    // Based on equation (24) of https://arxiv.org/pdf/2207.00669

    // Scaling parameter
    auto α = problem.DecisionVariable();

    auto distance = pose.Translation() - m_center;
    problem.SubjectTo(distance.Dot(distance) <= m_radius * m_radius * α * α);
    problem.SubjectTo(α <= 1.0);
  }

 private:
  trajopt::Translation2d m_center;
  double m_radius;
};

}  // namespace trajopt
