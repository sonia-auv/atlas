/**
 * \file	numbers.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_NUMBERS_H_
#define LIB_ATLAS_MATHS_NUMBERS_H_

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <type_traits>
#include <limits>
#include <lib_atlas/macros.h>
#include <lib_atlas/maths/trigo.h>

namespace atlas {

/**
 * Generate a random value between the two values passed in argument.
 *
 * \param low The minimum value that the random number should take.
 * \param high The maximum value that the random number should take.
 * \return A random number between low and high.
 */
template <class Tp_>
Tp_ Rand(const Tp_ &low = 0, const Tp_ &high = 1) ATLAS_NOEXCEPT;

/**
 *
 */
template <class Tp_>
Tp_ ProbabilityDistribution(const Tp_ &u, const Tp_ &s,
                            const Tp_ &x) ATLAS_NOEXCEPT;

/**
 * Clamp the value x into Xmin and Xmax.
 *
 * \param xmin The minimum value that x must take
 * \param xmax The maximum value that x must take.
 * \return The new value of x.
 */
template <typename Tp_>
Tp_ Clamp(const Tp_ &x, const Tp_ &xmin, const Tp_ &xmax) ATLAS_NOEXCEPT;

/**
 * Clamp x into the data set v.
 *
 * This will actuall call the clamp method with the minimum and the maximum
 * element found on the data set.
 *
 * \return The element x, clamped into the data set v.
 */
template <typename Tp_, typename Up_>
Tp_ Clamp(const Tp_ &x, const Up_ &v);

/**
 * \param v The variance
 */
double Gaussian(const double &x, const double &v) ATLAS_NOEXCEPT;

/**
 * \param v The variance
 */
double NormalizedGaussian(const double &x, const double &v) ATLAS_NOEXCEPT;

/**
 * This function will round the value to n significant digits.
 *
 * This can be usefull when you want to compare two numbers with a
 * certain precision.
 *
 * \param v The number to round.
 */
template <typename Tp_>
Tp_ SetPrecision(const Tp_ &v, uint32_t d) ATLAS_NOEXCEPT;

}  // namespace atlas

#include <lib_atlas/maths/numbers_inl.h>

#endif  // LIB_ATLAS_MATHS_NUMBERS_H_
