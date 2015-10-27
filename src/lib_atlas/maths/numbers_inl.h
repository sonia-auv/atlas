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

namespace atlas {

//------------------------------------------------------------------------------
//
template <class Tp_>
struct HasDecAccuracy {
  const static bool val = static_cast<Tp_>(0.5) > 0;
};

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double drand(bool force_re_seed = false,
                                 bool prevent_re_seed = false) ATLAS_NOEXCEPT {
  static bool seeded = prevent_re_seed;
  if (!seeded || force_re_seed) {
    srand(time(NULL));
    seeded = true;
  }

  return (double)rand() / RAND_MAX;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ Rand(const Tp_ &low, const Tp_ &high_,
                             const bool &nonzero = false) ATLAS_NOEXCEPT {
  const static bool dec_accuracy = HasDecAccuracy<Tp_>::val;
  const Tp_ high = dec_accuracy ? high_ : high_ + 1;

  Tp_ value;
  do {
    value = low + (high - low) * drand();
  } while (nonzero && value == 0 && !(low == high && low == 0));

  return value;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ vectorTo(const Tp_ &from_point,
                                 const Tp_ &to_point) ATLAS_NOEXCEPT {
  return VectorTo(from_point, to_point, from_point.x);
}

//------------------------------------------------------------------------------
//
template <class Tp_, class Up_>
Tp_ VectorTo(const Tp_ &from_point, const Tp_ &to_point,
             const Up_ point_data) ATLAS_NOEXCEPT {
  static Tp_ vec;
  static Up_ dist;
  static Up_ angle;

  dist = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
  vec.x = to_point.x - from_point.x;
  vec.y = to_point.y - from_point.y;

  if (vec.x == 0) {
    if (vec.y >= 0) {
      angle = static_cast<Tp_>(90);
    } else {
      angle = static_cast<Tp_>(270);
    }
  } else {
    angle = RadToDeg(fabs(atan(vec.y / vec.x)));
    if (vec.x >= 0 && vec.y >= 0) {
      //
    } else if (vec.x < 0 && vec.y >= 0) {
      angle = static_cast<Tp_>(180) - angle;
    } else if (vec.x < static_cast<Tp_>(0) && vec.y < static_cast<Tp_>(0)) {
      angle += static_cast<Tp_>(180);
    } else {
      angle = static_cast<Tp_>(360) - angle;
    }
  }
  NormalizeAngle(angle);

  vec.x = dist;
  vec.y = angle;

  return vec;
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
template <typename Tp_>
ATLAS_ALWAYS_INLINE void ClampPropValue(Tp_ &value1, Tp_ &value2,
                                        const Tp_ &mag_cap) {
  const static bool dec_accuracy = HasDecAccuracy<Tp_>::val;

  static Tp_ largest;
  static double scale;

  largest = fabs(value1) > fabs(value2) ? value1 : value2;

  if (fabs(largest) > fabs(mag_cap)) {
    scale = fabs((double)mag_cap) / (double)largest;

    value1 = dec_accuracy ? value1 * scale : round(value1 * scale);
    value2 = dec_accuracy ? value2 * scale : round(value2 * scale);
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE
    typename std::enable_if<std::is_floating_point<Tp_>::value, Tp_>::type
    Mod(const Tp_ &numerator, const Tp_ &denominator) ATLAS_NOEXCEPT {
  return fmod(numerator, denominator);
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE
    typename std::enable_if<!std::is_floating_point<Tp_>::value, Tp_>::type
    Mod(const Tp_ &numerator, const Tp_ &denominator) ATLAS_NOEXCEPT {
  return numerator % denominator;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double Gaussian(const double &x,
                                    const double &variance) ATLAS_NOEXCEPT {
  return (1 / sqrt(2 * M_PI * variance)) *
         pow(M_E, -pow(x, 2) / (2 * variance));
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double NormalizedGaussian(
    const double &x, const double &variance) ATLAS_NOEXCEPT {
  return pow(M_E, -pow(x, 2) / (2 * variance));
}

}  // namespace atlas
