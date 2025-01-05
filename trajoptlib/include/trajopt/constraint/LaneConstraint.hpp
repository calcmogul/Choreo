// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <optional>

#include "trajopt/constraint/HalfPlaneConstraint.hpp"
#include "trajopt/geometry/HPolytope2.hpp"
#include "trajopt/geometry/Rotation2.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Lane Constraint.
 *
 * Specifies the robot must stay between two lines.
 */
class TRAJOPT_DLLEXPORT LaneConstraint {
 public:
  /**
   * Constructs a LaneConstraint.
   *
   * @param centerLineStart Start point of the center line.
   * @param centerLineEnd End point of the center line.
   * @param tolerance Distance from center line to lane edge. Passing zero
   *   creates a line constraint.
   */
  LaneConstraint(Translation2d centerLineStart, Translation2d centerLineEnd,
                 double tolerance)
      : m_topLine{[&] {
          if (tolerance != 0.0) {
            double dx = centerLineEnd.X() - centerLineStart.X();
            double dy = centerLineEnd.Y() - centerLineStart.Y();
            double dist = std::hypot(dx, dy);
            auto offset = Translation2d{0.0, tolerance}.RotateBy(
                Rotation2d{dx / dist, dy / dist});

            return HalfPlaneConstraint{{0.0, 0.0},
                                       centerLineStart + offset,
                                       centerLineEnd + offset,
                                       Side::kBelow};
          } else {
            return HalfPlaneConstraint{
                {0.0, 0.0}, centerLineStart, centerLineEnd, Side::kOn};
          }
        }()},
        m_bottomLine{[&]() -> std::optional<HalfPlaneConstraint> {
          if (tolerance != 0.0) {
            double dx = centerLineEnd.X() - centerLineStart.X();
            double dy = centerLineEnd.Y() - centerLineStart.Y();
            double dist = std::hypot(dx, dy);
            auto offset = Translation2d{0.0, tolerance}.RotateBy(
                Rotation2d{dx / dist, dy / dist});

            return HalfPlaneConstraint{{0.0, 0.0},
                                       centerLineStart - offset,
                                       centerLineEnd - offset,
                                       Side::kAbove};
          } else {
            return std::nullopt;
          }
        }()} {}

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
             const HPolytope2v& robotRegion,
             const Translation2v& linearVelocity,
             const sleipnir::Variable& angularVelocity,
             const Translation2v& linearAcceleration,
             const sleipnir::Variable& angularAcceleration) {
    m_topLine.Apply(problem, pose, robotRegion, linearVelocity, angularVelocity,
                    linearAcceleration, angularAcceleration);
    if (m_bottomLine.has_value()) {
      m_bottomLine.value().Apply(problem, pose, robotRegion, linearVelocity,
                                 angularVelocity, linearAcceleration,
                                 angularAcceleration);
    }
  }

 private:
  HalfPlaneConstraint m_topLine;
  std::optional<HalfPlaneConstraint> m_bottomLine;
};

}  // namespace trajopt
