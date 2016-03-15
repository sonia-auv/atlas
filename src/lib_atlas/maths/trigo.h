/**
 * \file	trigo.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIB_ATLAS_MATHS_TRIGO_H_
#define LIB_ATLAS_MATHS_TRIGO_H_

#include <lib_atlas/macros.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <limits>
#include <type_traits>

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
