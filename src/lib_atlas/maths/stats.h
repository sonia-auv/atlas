/**
 * \file	stats.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Antoine Dozois <dozois.a@gmail.com>
 * \date	17/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_STATS_H_
#define ATLAS_MATHS_STATS_H_

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
auto euclidean(const Tp_ &v1, const Up_ &v2) -> double;

/**
 * Returns the Jaccard index of the two data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Jaccard_index
 *
 * \return The Jaccard index of v1 and v2.
 */
template <typename Tp_, typename Up_>
auto jaccard(const Tp_ &v1, const Up_ &v2) -> double;

/**
 * Returns the mean of the data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Mean
 *
 * \returns The means of the elements of v.
 */
template <typename Tp_>
auto mean(const Tp_ &v) ATLAS_NOEXCEPT -> double;

/**
 * Returns the median of the data set provided.
 *
 * This function will sort an find the value of the item in the middle.
 *
 * \returns The median of the vector vector_data.
 */
template <typename Tp_>
Tp_ median(std::vector<Tp_> const &v);

/**
 * Returns the geometric mean of the data set provided.
 *
 * This function give the nth root of the data, where the nth equal
 * the size of the vector.
 *
 * \returns The geometric mean of the vector vector_data.
 */
template <typename Tp_>
float geometric_mean(std::vector<Tp_> const &v);

/**
 * Returns the harmonic mean of the data set provided.
 *
 * The harmonic mean is useful to calculate the average rates of a
 * data set.
 *
 * \returns The harmonic mean of the vector vector_data.
 */
template <typename Tp_>
float harmonic_mean(std::vector<Tp_> const &v);

/**
 *
 */
template <typename Tp_>
auto min(const Tp_ &v) ATLAS_NOEXCEPT -> typename Tp_::value_type;

/**
 *
 */
template <typename Tp_>
auto max(const Tp_ &v) ATLAS_NOEXCEPT -> typename Tp_::value_type;

/**
 *
 */
template <typename Tp_>
auto clamp(const Tp_ &x, const Tp_ &xmin,
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
auto clamp(const Tp_ &x, const Up_ &v) -> decltype(clamp(x, min(v), max(v)));

/**
 *
 */
template <typename Tp_>
auto least_square(const Tp_ &v) -> std::array<double, 3>;

/**
 *
 */
template <typename Tp_>
auto predict(int i) -> typename Tp_::value_type;

/**
 * Returns the covariance of the two data set provided.
 *
 * Fore more informations:
 * https://en.wikipedia.org/wiki/Covariance
 *
 * \return The covariance of v1 and v2
 */
template <typename Tp_, typename Up_>
auto covariance(const Tp_ &v1, const Tp_ &v2) -> double;

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
auto std_dev(const Tp_ &v) ATLAS_NOEXCEPT -> double;

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
auto pearson(const Tp_ &v1, const Up_ &v2) -> double;

}  // namespace atlas

#include <lib_atlas/maths/stats_inl.h>

#endif  // ATLAS_MATHS_STATS_H_
