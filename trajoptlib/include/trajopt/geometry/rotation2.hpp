// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>
#include <concepts>
#include <numbers>
#include <utility>
#include <vector>

#include <sleipnir/autodiff/variable.hpp>

namespace trajopt {

/**
 * A rotation in a 2D coordinate frame represented by a point on the unit circle
 * (cosine and sine).
 */
template <typename T>
class Rotation2 {
 public:
  /**
   * Constructs a rotation with a default angle of 0 degrees.
   */
  constexpr Rotation2() = default;

  /**
   * Constructs a rotation with the given angle.
   *
   * @param angle The angle in radians.
   */
  constexpr Rotation2(const T& angle) {  // NOLINT
    using std::cos;
    using std::sin;

    m_cos = cos(angle);  // NOLINT
    m_sin = sin(angle);  // NOLINT
  }

  /**
   * Constructs a rotation with the given cosine and sine components.
   *
   * @param cos The cosine component of the rotation.
   * @param sin The sine component of the rotation.
   */
  constexpr Rotation2(T cos, T sin)
    requires(!std::same_as<T, double>)
      : m_cos{std::move(cos)}, m_sin{std::move(sin)} {}

  /**
   * Constructs a rotation with the given x and y components. The x and y don't
   * have to be normalized.
   *
   * @param x The x component of the rotation.
   * @param y The y component of the rotation.
   */
  constexpr Rotation2(double x, double y)
    requires std::same_as<T, double>
  {
    double magnitude = std::hypot(x, y);
    if (magnitude > 1e-6) {
      m_cos = x / magnitude;
      m_sin = y / magnitude;
    } else {
      m_cos = 1.0;
      m_sin = 0.0;
    }
  }

  /**
   * Coerces one rotation type into another.
   *
   * @param other The other rotation type.
   */
  template <typename U>
  constexpr explicit Rotation2(const Rotation2<U>& other)
      : m_cos{other.cos()}, m_sin{other.sin()} {}

  /**
   * Adds two rotations together, with the result being bounded between -pi and
   * pi.
   *
   * @param other The rotation to add.
   * @return The sum of the two rotations.
   */
  constexpr Rotation2<T> operator+(const Rotation2<T>& other) const {
    return rotate_by(other);
  }

  /**
   * Subtracts the new rotation from the current rotation and returns the new
   * rotation.
   *
   * @param other The rotation to subtract.
   * @return The difference between the two rotations.
   */
  constexpr Rotation2<T> operator-(const Rotation2<T>& other) const {
    return *this + -other;
  }

  /**
   * Takes the inverse of the current rotation.
   *
   * @return The inverse of the current rotation.
   */
  constexpr Rotation2<T> operator-() const {
    return Rotation2<T>{m_cos, -m_sin};
  }

  /**
   * Adds the new rotation to the current rotation using a clockwise rotation
   * matrix.
   *
   * @param other The rotation to rotate by.
   * @return The new rotated rotation.
   */
  template <typename U>
  constexpr Rotation2<U> rotate_by(const Rotation2<U>& other) const {
    using R = decltype(std::declval<T>() + std::declval<U>());
    return Rotation2<R>{cos() * other.cos() - sin() * other.sin(),   // NOLINT
                        cos() * other.sin() + sin() * other.cos()};  // NOLINT
  }

  /**
   * Returns the rotation as an Euler angle in radians.
   *
   * @return The Euler angle in radians.
   */
  constexpr T radians() const { return atan2(m_sin, m_cos); }  // NOLINT

  /**
   * Returns the rotation as an Euler angle in degrees.
   *
   * @return The Euler angle in degrees.
   */
  constexpr T degrees() const {
    return atan2(m_sin, m_cos) / std::numbers::pi * 180.0;  // NOLINT
  }

  /**
   * Returns the cosine of the rotation.
   *
   * @return The cosine of the rotation.
   */
  constexpr const T& cos() const { return m_cos; }  // NOLINT

  /**
   * Returns the sine of the rotation.
   *
   * @return The sine of the rotation.
   */
  constexpr const T& sin() const { return m_sin; }  // NOLINT

 private:
  T m_cos = 1.0;
  T m_sin = 0.0;
};

using Rotation2d = Rotation2<double>;
using Rotation2v = Rotation2<slp::Variable>;

template <typename T, typename U>
  requires std::same_as<T, slp::Variable> || std::same_as<U, slp::Variable>
slp::EqualityConstraints operator==(const Rotation2<T>& lhs,
                                    const Rotation2<U>& rhs) {
  std::vector<slp::EqualityConstraints> constraints;

  // Constrain angle equality on manifold.
  //
  // Let lhs = <cos(a), sin(a)>.  NOLINT
  // Let rhs = <cos(b), sin(b)>.  NOLINT
  //
  // If the angles are equal, the angle between the unit vectors should be
  // zero.
  //
  //   lhs x rhs = ‖lhs‖ ‖rhs‖ sin(angleBetween)  NOLINT
  //         = 1 * 1 * 0
  //         = 0
  //
  // NOTE: angleBetween = π rad would be another solution
  constraints.emplace_back(lhs.cos() * rhs.sin() - lhs.sin() * rhs.cos() ==
                           0.0);

  // Require that lhs and rhs are unit vectors if they contain autodiff
  // variables, since their values can change.
  if constexpr (std::same_as<T, slp::Variable>) {
    constraints.emplace_back(lhs.cos() * lhs.cos() + lhs.sin() * lhs.sin() ==
                             1.0);
  }
  if constexpr (std::same_as<U, slp::Variable>) {
    constraints.emplace_back(rhs.cos() * rhs.cos() + rhs.sin() * rhs.sin() ==
                             1.0);
  }

  return slp::EqualityConstraints{constraints};
}

inline bool operator==(const Rotation2d& lhs, const Rotation2d& rhs) {
  return lhs.cos() == rhs.cos() && lhs.sin() == rhs.sin();
}

}  // namespace trajopt
