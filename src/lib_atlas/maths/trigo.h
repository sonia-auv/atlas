/**
 * \file	trigo.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_TRIGO_H_
#define LIB_ATLAS_MATHS_TRIGO_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <type_traits>
#include <limits>
#include <lib_atlas/macros.h>

namespace atlas {

template <class Tp_>
Tp_ DegToRad(const Tp_ &degrees) ATLAS_NOEXCEPT;

template <class Tp_>
Tp_ RadToDeg(const Tp_ &radians) ATLAS_NOEXCEPT;

template <class Tp_>
void PolarToEuler(Tp_ &angle) ATLAS_NOEXCEPT;

template <class Tp_>
void EulerToPolar(Tp_ &angle) ATLAS_NOEXCEPT;

template <typename Tp_>
Tp_ AngleDistRel(const Tp_ &from_angle, const Tp_ &to_angle) ATLAS_NOEXCEPT;

template <typename Tp_>
void NormalizeAngle(Tp_ &angle) ATLAS_NOEXCEPT;

// calculates the ratio of difference between a1 and a2 such that angleRatio(x,
// x+180) = 1 and angleRatio(x, x) = 0
template <class Tp_>
Tp_ LinearAngleDiffRatio(const Tp_ &from_angle,
                         const Tp_ &to_angle) ATLAS_NOEXCEPT;

template <class Tp_>
Tp_ TrigAngleDiffRatio(const Tp_ &from_angle,
                       const Tp_ &to_angle) ATLAS_NOEXCEPT;

}  // namespace atlas

#include <lib_atlas/maths/trigo_inl.h>

#endif  // LIB_ATLAS_MATHS_TRIGO_H_
