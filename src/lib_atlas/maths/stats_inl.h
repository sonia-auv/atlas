/**
 * \file	stats_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Antoine Dozois <dozois.a@gmail.com>
 * \date	17/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_STATS_H_
#error This file may only be included from stats.h
#endif

#include <math.h>
#include <algorithm>

namespace atlas {

namespace details {

// Allow us to check if the type we received is actually iterable.
// If the data set must be iterable, we can static assert it with this
// type traits.
// Fore more informations:
// http://stackoverflow.com/questions/13830158/check-if-a-variable-is-iterable

//------------------------------------------------------------------------------
//
template <typename Tp_>
auto is_iterable_impl(int) -> decltype(
    std::begin(std::declval<Tp_ &>()) != std::end(std::declval<Tp_ &>()),
    ++std::declval<decltype(std::begin(std::declval<Tp_ &>())) &>(),
    *begin(std::declval<Tp_ &>()), std::true_type{});

template <typename Tp_>
std::false_type is_iterable_impl(...);

template <typename Tp_>
using is_iterable = decltype(is_iterable_impl<Tp_>(0));

}  // namespace details

//------------------------------------------------------------------------------
//
template <typename Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto euclidean(const Tp_ &v1, const Up_ &v2) -> double {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  static_assert(details::is_iterable<Up_>::value,
                "The data set must be iterable");
  if (v1.size() != v2.size()) {
    throw std::invalid_argument("The lengh of the data set is not the same");
  }

  typename Tp_::value_type diff = {v1[0] - v2[0]};
  typename Tp_::value_type s = {diff * diff};

  for (uint64_t i = 1; i < v1.size(); ++i) {
    diff = v1[i] - v2[i];
    s += diff * diff;
  }
  return sqrt(static_cast<double>(s));
}

//------------------------------------------------------------------------------
//
template <typename Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto jaccard(const Tp_ &v1, const Up_ &v2) -> double {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  static_assert(details::is_iterable<Up_>::value,
                "The data set must be iterable");
  if (v1.size() != v2.size()) {
    throw std::invalid_argument("The lengh of the data set is not the same");
  }

  uint64_t eq = {0};
  uint64_t nq = {0};

  for (uint64_t i = 0; i < v1.size(); ++i) {
    if (v1[i] == v2[i]) {
      if (v1[i] != 0) {
        ++eq;
      } else {
        ++nq;
      }
    }
  }
  return static_cast<double>(eq) / static_cast<double>(v1.size() - (nq + eq));
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE auto mean(const Tp_ &v) ATLAS_NOEXCEPT -> double {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  typename Tp_::value_type s = {0};
  for (const auto &e : v) {
    s += e;
  }
  return static_cast<double>(s) / static_cast<double>(v.size());
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
Tp_ median(std::vector<Tp_> const &v) {
  std::vector<Tp_> sorted_vector;
  sorted_vector = v;
  std::sort(sorted_vector.begin(), sorted_vector.end());
  return sorted_vector[ceil(sorted_vector.size() / 2)];
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
float geometric_mean(std::vector<Tp_> const &v) {
  double sum = 1.f;

  for (const auto &e : v) {
    sum *= e;
  }

  return static_cast<float>(pow(sum, 1.0f / static_cast<float>(v.size())));
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
float harmonic_mean(std::vector<Tp_> const &v) {
  float harmonic = 0.f;
  for (const auto &e : v) {
    harmonic += 1.f / static_cast<float>(e);
  }

  if (harmonic == 0.f) {
    throw std::logic_error("The harmonic equal zero! Can't return result");
  }

  return static_cast<float>(v.size()) / harmonic;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE auto min(const Tp_ &v) ATLAS_NOEXCEPT ->
    typename Tp_::value_type {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  return *std::min_element(v.cbegin(), v.cend());
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE auto max(const Tp_ &v) ATLAS_NOEXCEPT ->
    typename Tp_::value_type {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  return *std::max_element(v.cbegin(), v.cend());
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE auto clamp(const Tp_ &x, const Tp_ &xmin,
                               const Tp_ &xmax) ATLAS_NOEXCEPT -> Tp_ {
  return x < xmin ? xmin : (x > xmax ? xmax : x);
}

//------------------------------------------------------------------------------
//
template <typename Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto clamp(const Tp_ &x, const Up_ &v)
    -> decltype(clamp(x, min(v), max(v))) {
  static_assert(details::is_iterable<Up_>::value,
                "The data set must be iterable");
  return clamp(x, min(v), max(v));
}

//------------------------------------------------------------------------------
//
template <typename Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto covariance(const Tp_ &v1, const Up_ &v2) -> double {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  static_assert(details::is_iterable<Up_>::value,
                "The data set must be iterable");
  if (v1.size() != v2.size()) {
    throw std::invalid_argument("The lengh of the data set is not the same");
  }

  double m1 = mean(v1);
  double m2 = mean(v2);
  double s =
      (static_cast<double>(v1[0]) - m1) * (static_cast<double>(v2[0]) - m2);

  for (uint64_t i = 1; i < v1.size(); ++i) {
    s += (static_cast<double>(v1[i]) - m1) * (static_cast<double>(v2[i]) - m2);
  }
  return s / static_cast<double>(v1.size() - 1);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE auto std_dev(const Tp_ &v) ATLAS_NOEXCEPT -> double {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  return sqrt(covariance(v, v));
}

//------------------------------------------------------------------------------
//
template <typename Tp_, typename Up_>
ATLAS_ALWAYS_INLINE auto pearson(const Tp_ &v1, const Up_ &v2) -> double {
  static_assert(details::is_iterable<Tp_>::value,
                "The data set must be iterable");
  static_assert(details::is_iterable<Up_>::value,
                "The data set must be iterable");
  double std_dev1 = std_dev(v1);
  double std_dev2 = std_dev(v2);

  if (std_dev1 * std_dev2 == 0) {
    throw std::invalid_argument("The standart deviation of these set is null.");
  }
  return covariance(v1, v2) / (std_dev1 * std_dev2);
}

}  // namespace.h
