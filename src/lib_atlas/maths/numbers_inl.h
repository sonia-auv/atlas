/**
 * \file	numbers_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_NUMBERS_H_
#error This file may only be included numbers.h
#endif  // LIB_ATLAS_MATHS_NUMBERS_H_

#include <cstdlib>
#include <random>

namespace atlas {

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ Rand(const Tp_ &low, const Tp_ &high) ATLAS_NOEXCEPT {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<Tp_> dist(low, high);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ ProbabilityDistribution(const Tp_ &u, const Tp_ &s,
                                                const Tp_ &x) ATLAS_NOEXCEPT {
  return exp(-pow(x - u, 2) / (2 * pow(s, 2))) / sqrt(2 * M_PI * pow(s, 2));
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double Gaussian(const double &x,
                                    const double &v) ATLAS_NOEXCEPT {
  return (1 / sqrt(2 * M_PI * v)) *
         exp(-pow(x, 2) / (2 * v));
}


//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double NormalizedGaussian(
    const double &x, const double &v) ATLAS_NOEXCEPT {
  return exp(-pow(x, 2) / (2 * v));
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ SetPrecision(const Tp_ &v, uint32_t d) ATLAS_NOEXCEPT {
  if (v == 0.0){
    return static_cast<Tp_>(0.0);
  }
  Tp_ factor = pow(10.0, d - ceil(log10(fabs(v))));
  return round(v * factor) / factor;
}

}  // namespace atlas
