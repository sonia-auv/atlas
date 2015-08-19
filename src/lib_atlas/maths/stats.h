/**
 * \file	stats.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	17/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_STATS_H_
#define ATLAS_MATHS_STATS_H_

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
template <class Tp_, typename Up_>
auto euclidean(const Tp_<Up_> &v1, const Tp_<Up_> &v2) -> double;

/**
 * Returns the Jaccard index of the two data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Jaccard_index
 *
 * \return The Jaccard index of v1 and v2.
 */
template <class Tp_, typename Up_>
auto jaccard(const Tp_<Up_> &v1, const Tp_<Up_> &v2) -> double;

/**
 * Returns the mean of the data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Mean
 *
 * \returns The means of the elements of v.
 */
template <class Tp_, typename Up_>
auto mean(const Tp_<Up_> &v) ATLAS_NOEXCEPT -> double;

/**
 * Returns the covariance of the two data set provided.
 *
 * Fore more informations:
 * https://en.wikipedia.org/wiki/Covariance
 *
 * \return The covariance of v1 and v2
 */
template <class Tp_, typename Up_>
auto covariance(const Tp_<Up_> &v1,
                const Tp_<Up_> &v2) ATLAS_NOEXCEPT -> double;

/**
 * Returns the standard deviation of the provided set.
 *
 * For performing the standard deviation, we calculate the covariance of
 * v1 with itself.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Standard_deviation
 */
template <class Tp_, typename Up_>
auto std_dev(const Tp_<Up_> &v) -> double;

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
template <class Tp_, typename Up_>
auto pearson(const Tp_<Up_> &v1, const Tp_<Up_> &v2) -> double;

}  // namespace atlas

#include <lib_atlas/maths/stats_inl.h>

#endif  // ATLAS_MATHS_STATS_H_
