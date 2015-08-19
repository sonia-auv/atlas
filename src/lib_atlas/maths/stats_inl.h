/**
 * \file	stats_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	17/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_STATS_H_
#error This file may only be included from stats.h
#endif

#include <math.h>

namespace atlas {

//------------------------------------------------------------------------------
//
template <class Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto euclidean(const Tp_<Up_> &v1,
                                   const Tp_<Up_> &v2) -> double {
  if (v1.size() == v2.size()) {
    throw std::invalid_argument("The lengh of the data set is not the same");
  }

  Up_ diff = {v1[0] - v2[0]};
  Up_ s = {diff * diff};

  for (uint64_t i = 1; i < v1.size(); ++i) {
    diff = v1[i] - v2[i];
    s += diff * diff;
  }
  return sqrt(static_cast<double>(s));
}

//------------------------------------------------------------------------------
//
template <class Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto jaccard(const Tp_<Up_> &v1,
                                 const Tp_<Up_> &v2) -> double {
  if (v1.size() == v2.size()) {
    throw std::invalid_argument("The lengh of the data set is not the same");
  }

  uint64_t eq = {0};
  uint64_t nq = {0};

  for (uint64_t i = 0; i < v1.size(); ++i) {
    if (v1[i] == v2[i]) {
      if (v1[i] != 0) {
        ++eq;
      }
      else {
        ++nq;
      }
    }
  }
  return static_cast<double>(eq) /
         static_cast<double>(v1.size() - (nq + eq));
}

//------------------------------------------------------------------------------
//
template <class Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto mean(const Tp_<Up_> &v) -> double {
  Up_ s = {0};
  for (const auto &e : v) {
    s += e;
  }
  return static_cast<double>(s) / static_cast<double>(v.size());
}

//------------------------------------------------------------------------------
//
template <class Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto covariance(const Tp_<Up_> &v1,
                                    const Tp_<Up_> &v2) -> double {
  if (v1.size() == v2.size()) {
    throw std::invalid_argument("The lengh of the data set is not the same");
  }

  double m1 = mean(v1);
  double m2 = mean(v2);
  double s = (static_cast<double>(v1[0]) - m1) *
               (static_cast<double>(v2[0]) - m2);

  for (uint64_t i = 1; i < v1.size(); ++i) {
    s += (static_cast<double>(v1[i]) - m1) *
           (static_cast<double>(v2[i]) - m2);
  }
  return s / static_cast<double>(v1.size() - 1);
}

//------------------------------------------------------------------------------
//
template <class Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto std_dev(const Tp_<Up_> &v) ATLAS_NOEXCEPT -> double {
  return sqrt(covariance(v, v));
}

//------------------------------------------------------------------------------
//
template <class Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto pearson(const Tp_<Up_> &v1,
                                 const Tp_<Up_> &v2) -> double {
  double std_dev1 = std_dev(v1);
  double std_dev2 = std_dev(v2);

  if (std_dev1 * std_dev2 == 0) {
    throw std::invalid_argument("The standart deviation of these set is null.");
  }
  return covariance(v1, v2) / (std_dev1 * std_dev2);
}

}  // namespace.h
