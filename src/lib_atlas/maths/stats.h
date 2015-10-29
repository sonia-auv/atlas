/**
 * \file	stats.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Antoine Dozois <dozois.a@gmail.com>
 * \date	17/08/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
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
double Euclidean(const Tp_ &v1, const Up_ &v2);

/**
 * Returns the Jaccard index of the two data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Jaccard_index
 *
 * \return The Jaccard index of v1 and v2.
 */
template <typename Tp_, typename Up_>
double Jaccard(const Tp_ &v1, const Up_ &v2);

/**
 * Returns the mean of the data set provided.
 *
 * For more informations:
 * https://en.wikipedia.org/wiki/Mean
 *
 * \returns The means of the elements of v.
 */
template <typename Tp_>
double Mean(const Tp_ &v) ATLAS_NOEXCEPT;

/**
 * Returns the median of the data set provided.
 *
 * This function will sort an find the value of the item in the middle.
 *
 * \returns The median of the vector vector_data.
 */
template <typename Tp_>
typename Tp_::value_type Median(const Tp_ &v) ATLAS_NOEXCEPT;

/**
 * Returns the geometric mean of the data set provided.
 *
 * This function give the nth root of the data, where the nth equal
 * the size of the vector.
 *
 * \returns The geometric mean of the vector vector_data.
 */
template <typename Tp_>
double GeometricMean(const Tp_ &v) ATLAS_NOEXCEPT;

/**
 * Returns the harmonic mean of the data set provided.
 *
 * The harmonic mean is useful to calculate the average rates of a
 * data set.
 *
 * \returns The harmonic mean of the vector vector_data.
 */
template <typename Tp_>
double HarmonicMean(const Tp_ &v);

/**
 * Return the minimum value in the iterable parameter v.
 *
 * \param v The collection to iterate to find the min value.
 * \return The min value in v.
 */
template <typename Tp_>
typename Tp_::value_type Min(const Tp_ &v) ATLAS_NOEXCEPT;

/**
 * Return the maximum value in the iterable parameter v.
 *
 * \param v The collection to iterate to find the max value.
 * \return The max value in v.
 */
template <typename Tp_>
typename Tp_::value_type Max(const Tp_ &v) ATLAS_NOEXCEPT;

/**
 *
 */
template <typename Tp_>
std::array<double, 3> LeastSquare(const Tp_ &v);

/**
 *
 */
template <typename Tp_>
typename Tp_::value_type Predict(int i);

/**
 * Returns the covariance of the two data set provided.
 *
 * Fore more informations:
 * https://en.wikipedia.org/wiki/Covariance
 *
 * \return The covariance of v1 and v2
 */
template <typename Tp_, typename Up_>
double Covariance(const Tp_ &v1, const Tp_ &v2);

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
double StdDeviation(const Tp_ &v) ATLAS_NOEXCEPT;

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
double Pearson(const Tp_ &v1, const Up_ &v2);

}  // namespace atlas

#include <lib_atlas/maths/stats_inl.h>

#endif  // LIB_ATLAS_MATHS_STATS_H_
