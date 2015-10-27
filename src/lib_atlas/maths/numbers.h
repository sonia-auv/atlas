/**
 * \file	numbers.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_NUMBERS_H_
#define LIB_ATLAS_MATHS_NUMBERS_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <type_traits>
#include <limits>
#include <lib_atlas/macros.h>
#include <lib_atlas/maths/trigo.h>

namespace atlas {

// by default: seed the random number generator if we haven't already done so,
// then return a number between 0 and 1
// if force_re_seed is used, the number generator is re-seeded
// if prevent_re_seed is used, number generator is never re-seeded; note that
// this condition only applies on the first call of this function
double drand(bool force_re_seed = false,
             bool prevent_re_seed = false) ATLAS_NOEXCEPT;

// return a random number between "low" and "high"; if "nonzero" is true then
// this value will never be zero
template <class Tp_>
Tp_ Rand(const Tp_ &low = 0, const Tp_ &high_ = 1,
         const bool &nonzero = false) ATLAS_NOEXCEPT;

// Tp_ must have publicly-accessible members "x" and "y"
template <class Tp_>
Tp_ vectorTo(const Tp_ &from_point, const Tp_ &to_point) ATLAS_NOEXCEPT;

// Tp_ must have publicly-accessible members "x" and "y"
// point_data is used here solely to deduce Up_
template <class Tp_, class Up_>
Tp_ VectorTo(const Tp_ &from_point, const Tp_ &to_point,
             const Up_ point_data) ATLAS_NOEXCEPT;

template <class Tp_>
Tp_ ProbabilityDistribution(const Tp_ &u, const Tp_ &s,
                            const Tp_ &x) ATLAS_NOEXCEPT;

template <typename Tp_>
void ClampPropValue(Tp_ &value1, Tp_ &value2,
                    const Tp_ &mag_cap) ATLAS_NOEXCEPT;

template <class Tp_>
typename std::enable_if<std::is_floating_point<Tp_>::value, Tp_>::type Mod(
    const Tp_ &numerator, const Tp_ &denominator) ATLAS_NOEXCEPT;

template <class Tp_>
typename std::enable_if<!std::is_floating_point<Tp_>::value, Tp_>::type Mod(
    const Tp_ &numerator, const Tp_ &denominator) ATLAS_NOEXCEPT;

double Gaussian(const double &x, const double &variance) ATLAS_NOEXCEPT;

double NormalizedGaussian(const double &x,
                          const double &variance) ATLAS_NOEXCEPT;

}  // namespace atlas

#include <lib_atlas/maths/numbers_inl.h>

#endif  // LIB_ATLAS_MATHS_NUMBERS_H_
