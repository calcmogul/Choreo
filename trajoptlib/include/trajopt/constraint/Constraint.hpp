// Copyright (c) TrajoptLib contributors

#pragma once

#include <concepts>
#include <variant>

#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "trajopt/constraint/AngularVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/HalfPlaneConstraint.hpp"
#include "trajopt/constraint/KeepInCircleConstraint.hpp"
#include "trajopt/constraint/KeepOutCircleConstraint.hpp"
#include "trajopt/constraint/LaneConstraint.hpp"
#include "trajopt/constraint/LinearAccelerationMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/LinearVelocityDirectionConstraint.hpp"
#include "trajopt/constraint/LinearVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/PointAtConstraint.hpp"
#include "trajopt/constraint/PoseEqualityConstraint.hpp"
#include "trajopt/constraint/TranslationEqualityConstraint.hpp"
#include "trajopt/geometry/HPolytope2.hpp"
#include "trajopt/geometry/Pose2.hpp"
#include "trajopt/geometry/Translation2.hpp"

namespace trajopt {

/**
 * ConstraintType concept.
 *
 * To make TrajoptLib support a new constraint type, do the following in this
 * file:
 *
 * 1. Include the type's header file
 * 2. Add a constraint static assert for the type
 * 3. Add the type to Constraint's std::variant type list
 */
template <typename T>
concept ConstraintType = requires(
    T self, sleipnir::OptimizationProblem& problem, const Pose2v& pose,
    const HPolytope2v& robotRegion, const Translation2v& linearVelocity,
    const sleipnir::Variable& angularVelocity,
    const Translation2v& linearAcceleration,
    const sleipnir::Variable& angularAcceleration) {
  {
    self.Apply(problem, pose, robotRegion, linearVelocity, angularVelocity,
               linearAcceleration, angularAcceleration)
  } -> std::same_as<void>;
};

static_assert(ConstraintType<AngularVelocityMaxMagnitudeConstraint>);
static_assert(ConstraintType<HalfPlaneConstraint>);
static_assert(ConstraintType<KeepInCircleConstraint>);
static_assert(ConstraintType<KeepOutCircleConstraint>);
static_assert(ConstraintType<LaneConstraint>);
static_assert(ConstraintType<LinearAccelerationMaxMagnitudeConstraint>);
static_assert(ConstraintType<LinearVelocityDirectionConstraint>);
static_assert(ConstraintType<LinearVelocityMaxMagnitudeConstraint>);
static_assert(ConstraintType<PointAtConstraint>);
static_assert(ConstraintType<PoseEqualityConstraint>);
static_assert(ConstraintType<TranslationEqualityConstraint>);

using Constraint = std::variant<
    // clang-format off
    AngularVelocityMaxMagnitudeConstraint,
    KeepInCircleConstraint,
    KeepOutCircleConstraint,
    LaneConstraint,
    LinearAccelerationMaxMagnitudeConstraint,
    LinearVelocityDirectionConstraint,
    LinearVelocityMaxMagnitudeConstraint,
    PointAtConstraint,
    HalfPlaneConstraint,
    PoseEqualityConstraint,
    TranslationEqualityConstraint
    // clang-format on
    >;

}  // namespace trajopt
