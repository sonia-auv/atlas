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
  return degrees * (M_PI / 180);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ RadToDeg(const Tp_ &radians) ATLAS_NOEXCEPT {
  return radians * (180 / M_PI);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ AngleDistance(const Tp_ &a1,
                                      const Tp_ &a2) ATLAS_NOEXCEPT {
  Tp_ angle_diff = a1 - a2;

  if (angle_diff < static_cast<Tp_>(-180)) {
    angle_diff += static_cast<Tp_>(-360);
  } else if (angle_diff > static_cast<Tp_>(180)) {
    angle_diff -= static_cast<Tp_>(360);
  }
  return angle_diff;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ NormalizeAngle(const Tp_ &angle) ATLAS_NOEXCEPT {
  auto norm = fmod(angle, static_cast<Tp_>(360));
  return norm < 0 ? 360 + norm : norm;
}

}  // namespace atlas
