/**
 * \file	trigo.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
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

/**
 * Convert the value passed in argument into a radian value.
 *
 * This assume that the value passed is in degree and will convert
 * it into a radian value.
 *
 * \param degree The value of the angle to convert.
 * \return The angle converted in radian.
 */
template <class Tp_>
Tp_ DegToRad(const Tp_ &degrees) ATLAS_NOEXCEPT;

/**
 * Convert the value passed in argument into a radian value.
 *
 * This assume that the value passed is in degree and will convert
 * it into a radian value.
 *
 * \param degree The value of the angle to convert.
 * \return The angle converted in radian.
 */
template <class Tp_>
Tp_ RadToDeg(const Tp_ &radians) ATLAS_NOEXCEPT;

/**
 * Normalize an angle in the inteval of [0; 360[
 *
 * This take a degree angle and apply a modulus operation in order
 * to have the value clamped between 0 and 360.
 *
 * \param a The angle to clamp
 * \return The normalized angle value.
 */
template <typename Tp_>
Tp_ NormalizeAngle(const Tp_ &angle) ATLAS_NOEXCEPT;

}  // namespace atlas

#include <lib_atlas/maths/trigo_inl.h>

#endif  // LIB_ATLAS_MATHS_TRIGO_H_
