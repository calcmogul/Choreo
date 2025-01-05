// Copyright (c) TrajoptLib contributors

#pragma once

#include <utility>
#include <vector>

#include <sleipnir/autodiff/Variable.hpp>

#include "trajopt/geometry/Translation2.hpp"

namespace trajopt {

/**
 * Represents a 2D H-polytope of the form Ax ≤ b where x is an x-y pair.
 */
template <typename T>
class HPolytope2 {
 public:
  /**
   * Constructs a 2D H-polytope from a list of 2D points.
   *
   * @param points The list of 2D points wound clockwise around the polytope
   *     interior.
   */
  template <typename U>
  explicit HPolytope2(const std::vector<Translation2<U>>& points)
      : m_A{points.size(), 2}, m_b{points.size()} {
    // Rearrange y − y₀ ≤ m(x − x₀) where m = (y₁ − y₀)/(x₁ − x₀) into
    // ax + by ≤ c form.
    //
    //   y − y₀ ≤ m(x − x₀)
    //   y − y₀ ≤ (y₁ − y₀)/(x₁ − x₀)(x − x₀)
    //   (x₁ − x₀)(y − y₀) ≤ (y₁ − y₀)(x − x₀)
    //   (x₁ − x₀)y − (x₁ − x₀)y₀ ≤ (y₁ − y₀)x − (y₁ − y₀)x₀
    //   −(y₁ − y₀)x + (x₁ − x₀)y ≤ −(y₁ − y₀)x₀ + (x₁ − x₀)y₀
    //
    // Let a = −(y₁ − y₀) and b = x₁ − x₀.
    //
    //   ax + by ≤ ax₀ + by₀
    //
    // Let c = ax₀ + by₀.
    //
    //   ax + by ≤ c
    for (size_t i = 0; i < points.size(); ++i) {
      size_t j = (i + 1) % points.size();

      auto diff = points[j] - points[i];

      double a = -diff.Y();
      double b = diff.X();
      double c = a * points[i].X() + b * points[i].Y();

      m_A(i, 0) = a;
      m_A(i, 1) = b;
      m_b(i) = c;
    }
  }

  /**
   * Constructs a 2D H-polytope from an A matrix and b vector.
   *
   * @param A The A matrix.
   * @param b The b vector.
   */
  HPolytope2(Eigen::Matrix<T, Eigen::Dynamic, 2> A,
             Eigen::Vector<T, Eigen::Dynamic> b)
      : m_A{std::move(A)}, m_b{std::move(b)} {}

  /**
   * Coerces one H-polytope type into another.
   *
   * @param other The other H-polytope type.
   */
  template <typename U>
  explicit HPolytope2(const HPolytope2<U>& other)
      : m_A{other.m_A}, m_b{other.m_b} {}

  /**
   * Returns the A matrix in Ax ≤ b.
   *
   * @return The A matrix in Ax ≤ b.
   */
  const Eigen::Matrix<T, Eigen::Dynamic, 2>& A() const { return m_A; }

  /**
   * Returns the b vector in Ax ≤ b.
   *
   * @return The b vector in Ax ≤ b.
   */
  const Eigen::Vector<T, Eigen::Dynamic>& b() const { return m_b; }

  /**
   * Applies a rotation to the polytope in 2D space.
   *
   * @param other The rotation to rotate the polytope by.
   * @return The new rotated polytope.
   */
  template <typename U>
  auto RotateBy(const Rotation2<U>& other) const {
    using R = decltype(std::declval<T>() + std::declval<U>());
    return HPolytope2<R>{m_A * other.ToMatrix().transpose(), m_b};
  }

  /**
   * Returns true if the polytope contains the point.
   *
   * @param point The point.
   * @return True if the polytope contains the point.
   */
  bool Contains(const Translation2<T>& point) const {
    Eigen::Vector2d x{{point.X()}, {point.Y()}};

    for (int row = 0; row < m_b.rows(); ++row) {
      if (m_A.template block<1, 2>(row, 0) * x > m_b(row)) {
        return false;
      }
    }

    return true;
  }

 private:
  // Representation for the H-polytope Ax ≤ b where x is x-y pair
  Eigen::Matrix<T, Eigen::Dynamic, 2> m_A;
  Eigen::Vector<T, Eigen::Dynamic> m_b;
};

using HPolytope2d = HPolytope2<double>;
using HPolytope2v = HPolytope2<sleipnir::Variable>;

}  // namespace trajopt
