/**
 * \file	stats.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	17/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_STATS_H_
#define LIB_ATLAS_MATHS_STATS_H_

#include <array>
#include <lib_atlas/macros.h>

namespace atlas {

/**
 * Returns the euclidean distance between the two data sets.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Euclidean_distance
 *
 * \return The euclidean distance of v1 and v2.
 */
template <typename Tp_, typename Up_>
auto Euclidean(const Tp_ &v1, const Up_ &v2) -> double;

/**
 * Returns the Jaccard index of the two data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Jaccard_index
 *
 * \return The Jaccard index of v1 and v2.
 */
template <typename Tp_, typename Up_>
auto Jaccard(const Tp_ &v1, const Up_ &v2) -> double;

/**
 * Returns the mean of the data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Mean
 *
 * \returns The means of the elements of v.
 */
template <typename Tp_>
auto Mean(const Tp_ &v) ATLAS_NOEXCEPT -> double;

/**
 *
 */
template <typename Tp_>
auto Min(const Tp_ &v) ATLAS_NOEXCEPT -> typename Tp_::value_type;

/**
 *
 */
template <typename Tp_>
auto Max(const Tp_ &v) ATLAS_NOEXCEPT -> typename Tp_::value_type;

/**
 *
 */
template <typename Tp_>
auto Clamp(const Tp_ &x, const Tp_ &xmin,
           const Tp_ &xmax) ATLAS_NOEXCEPT -> Tp_;

/**
 * Clamp x into the data set v.
 *
 * This will actuall call the clamp method with the minimum and the maximum
 * element found on the data set.
 *
 * \return The element x, clamped into the data set v.
 */
template <typename Tp_, typename Up_>
auto Clamp(const Tp_ &x, const Up_ &v) -> decltype(Clamp(x, Min(v), Max(v)));

/**
 *
 */
template <typename Tp_>
auto LeastSquare(const Tp_ &v) -> std::array<double, 3>;

/**
 *
 */
template <typename Tp_>
auto Predict(int i) -> typename Tp_::value_type;

/**
 * Returns the covariance of the two data set provided.
 *
 * Fore more informations:
 * https://en.wikipedia.org/wiki/Covariance
 *
 * \return The covariance of v1 and v2
 */
template <typename Tp_, typename Up_>
auto Covariance(const Tp_ &v1, const Tp_ &v2) -> double;

/**
 * Returns the standard deviation of the provided set.
 *
 * For performing the standard deviation, we calculate the covariance of
 * v1 with itself.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Standard_deviation
 */
template <typename Tp_>
auto StdDeviation(const Tp_ &v) ATLAS_NOEXCEPT -> double;

/**
 * Returns the Pearson correlation coefficient.
 *
 * The result of this function is in the set [-1; 1] with the value 0 meaning
 * that the sets of numbers provided are not linear, -1 and 1 meaning that
 * the sets of numbers follow a linear relation.
 * For more informations:
 * https://en.wikipedia.org/wiki/Pearson_product-moment_correlation_coefficient
 *
 * \return The Pearson product-moment correlation coefficient of the provided
 * sets.
 */
template <typename Tp_, typename Up_>
auto Pearson(const Tp_ &v1, const Up_ &v2) -> double;

}  // namespace atlas

#include <lib_atlas/maths/stats_inl.h>

#endif  // LIB_ATLAS_MATHS_STATS_H_
