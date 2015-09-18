/**
 * \file	averaging.h
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	8/09/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_AVERAGING_H_
#define ATLAS_MATHS_AVERAGING_H_

#include <vector>

namespace atlas {

/**
 * Returns the mean of the data set provided.
 *
 * \returns The means of the vector vector_data.
 */
template <typename Data>
double Mean(std::vector<Data> const &vector_data);

/**
 * Returns the median of the data set provided.
 *
 * This function will sort an find the value of the item in the middle.
 *
 * \returns The median of the vector vector_data.
 */
template <typename Data>
Data Median(std::vector<Data> const &vector_data);

/**
 * Returns the geometric mean of the data set provided.
 *
 * This function give the nth root of the data, where the nth equal
 * the size of the vector.
 *
 * \returns The geometric mean of the vector vector_data.
 */
template <typename Data>
float GeometricMean(std::vector<Data> const &vector_data);

/**
 * Returns the harmonic mean of the data set provided.
 *
 * The harmonic mean is useful to calculate the average rates of a
 * data set.
 *
 * \returns The harmonic mean of the vector vector_data.
 */
template <typename Data>
float HarmonicMean(std::vector<Data> const &vector_data);
}

#include <lib_atlas/maths/averaging_inl.h>

#endif  // LIB_ATLAS_AVERAGIN_H
