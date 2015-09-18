/**
 * \file	averaging_inl.h
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	8/09/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_AVERAGING_H_
#error This file may only be included averaging.h
#endif

#include <algorithm>
#include <math.h>

namespace atlas {

template <typename Data>
double Mean(std::vector<Data> const &vector_data) {
  double sum = 0;
  for (int i = 0; i < vector_data.size(); ++i) {
    sum += vector_data[i];
  }

  return sum / vector_data.size();
}

//------------------------------------------------------------------------------
//
template <typename Data>
Data Median(std::vector<Data> const &vector_data) {
  std::vector<Data> sorted_vector;
  sorted_vector = vector_data;
  std::sort(sorted_vector.begin(), sorted_vector.end());
  return sorted_vector[ceil(sorted_vector.size() / 2)];
}

//------------------------------------------------------------------------------
//
template <typename Data>
float GeometricMean(std::vector<Data> const &vector_data) {
  double sum = 1.f;

  for (const auto &data : vector_data) {
    sum *= data;
  }

  return static_cast<float>(pow(sum, 1.0f / static_cast<float>(vector_data.size())));
}

//------------------------------------------------------------------------------
//
template <typename Data>
float HarmonicMean(std::vector<Data> const &vector_data) {
  float harmonic = 0.f;
  for (const auto &data : vector_data) {
    harmonic += 1.f / static_cast<float>(data);
  }

  if (harmonic == 0.f) {
    throw std::logic_error("The harmonic equal zero! Can't return result");
  }

  return static_cast<float>(vector_data.size()) / harmonic;
}
}
