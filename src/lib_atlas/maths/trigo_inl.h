/**
 * \file	trigo_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_TRIGO_H_
#error This file may only be included trigo.h
#endif  // LIB_ATLAS_MATHS_TRIGO_H_

namespace atlas {

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ DegToRad(const Tp_ &degrees) ATLAS_NOEXCEPT {
  constexpr static double conversion_ratio = M_PI / 180;
  return degrees * conversion_ratio;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ RadToDeg(const Tp_ &radians) ATLAS_NOEXCEPT {
  constexpr static double conversion_ratio = 180 / M_PI;
  return radians * conversion_ratio;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE void PolarToEuler(Tp_ &angle) ATLAS_NOEXCEPT {
  angle += static_cast<Tp_>(-90) + (angle < static_cast<Tp_>(0)
                                        ? static_cast<Tp_>(360)
                                        : static_cast<Tp_>(0));
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE void EulerToPolar(Tp_ &angle) ATLAS_NOEXCEPT {
  angle += static_cast<Tp_>(90) + (angle < static_cast<Tp_>(0)
                                       ? static_cast<Tp_>(360)
                                       : static_cast<Tp_>(0));
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ AngleDistRel(const Tp_ &from_angle,
                                     const Tp_ &to_angle) ATLAS_NOEXCEPT {
  static Tp_ angle_diff;

  angle_diff = from_angle - to_angle;

  // this line does the following:
  // if c < -180, subtract 360 from a2; else if c > 180, add 360 to a2; else add
  // 0 to a2
  // return a2 - angle1
  return (to_angle +
          (angle_diff < static_cast<Tp_>(-180)
               ? static_cast<Tp_>(-360)
               : (angle_diff > static_cast<Tp_>(180) ? static_cast<Tp_>(360)
                                                     : static_cast<Tp_>(0)))) -
         from_angle;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void NormalizeAngle(Tp_ &angle) ATLAS_NOEXCEPT {
  angle = fmod(angle, static_cast<Tp_>(360));
}

//------------------------------------------------------------------------------
//
// calculates the ratio of difference between a1 and a2 such that angleRatio(x,
// x+180) = 1 and angleRatio(x, x) = 0
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ LinearAngleDiffRatio(
    const Tp_ &from_angle, const Tp_ &to_angle) ATLAS_NOEXCEPT {
  static Tp_ result;
  result = fabs(from_angle - to_angle);

  while (result > static_cast<Tp_>(360)) {
    result -= static_cast<Tp_>(360);
  }
  while (result < static_cast<Tp_>(0)) {
    result += static_cast<Tp_>(360);
  }
  if (result > static_cast<Tp_>(180)) {
    result = static_cast<Tp_>(360) - result;
  }
  result /= static_cast<Tp_>(180);

  return result;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ TrigAngleDiffRatio(const Tp_ &from_angle,
                                           const Tp_ &to_angle) ATLAS_NOEXCEPT {
  static Tp_ from_angle_rad;
  static Tp_ to_angle_rad;
  static Tp_ d;

  from_angle_rad = DegToRad(from_angle);
  to_angle_rad = DegToRad(to_angle);
  d = sqrt(pow(cos(from_angle_rad) - cos(to_angle_rad), 2) +
           pow(sin(from_angle_rad) - sin(to_angle_rad), 2));

  return d / static_cast<Tp_>(2);
}

}  // namespace atlas
