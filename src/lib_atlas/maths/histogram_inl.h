/**
 * \file	histogram_inl.h
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	8/09/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_MATHS_HISTOGRAM_H_
#error This file may only be included histogram.h
#endif  // LIB_ATLAS_HISTOGRAM_INL_H

#include <algorithm>
#include <lib_atlas/maths/stats.h>

namespace atlas {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data,
                                               double inter)
    : min_data_(min(data)),
      max_data_(max(data)),
      inter_(inter),
      inter_func_(false),
      pfunc_inter_(NULL),
      data_(data),
      histogram_(NULL),
      size_(max_data_ - min_data_) {}

//------------------------------------------------------------------------------
//

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data,
                                               unsigned int function)
    : min_data_(min),
      max_data_(max),
      min_histogram_(0),
      max_histogram_(0),
      pfunc_inter_((unsigned int (*)(unsigned int))function),
      inter_func_(true),
      inter_(0),
      data_(data),
      histogram_(NULL),
      size_(max_data_ - min_data_) {}

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::~Histogram() {}
//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//

template <typename Data>
inline void Histogram<Data>::CreateHistogram() {
  std::sort(data_.begin(), data_.end());
  std::vector<std::tuple<int, Data>> histo_init_ = {
      std::make_tuple(1, data_[0])};

  int i = 0;
  for (typename std::vector<Data>::iterator it = std::next(data_.begin());
       it != (data_.end()+1); it++) {
    if (*it == std::get<1>(histo_init_[i])) {
      std::get<0>(histo_init_[i]) += 1;
    } else if (*it - std::get<1>(histo_init_[i]) == 1) {
      histo_init_.push_back(std::make_tuple(1, *it));
      i++;
    } else {
      for (int j = std::get<1>(histo_init_[i]); j < *it; j++) {
        histo_init_.push_back(std::make_tuple(0, j+1));
      }
      i += (*it - std::get<1>(histo_init_[i]));
      std::get<0>(histo_init_[i]) += 1;
    }

    histogram_ = histo_init_;
  }
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline Data Histogram<Data>::GetMaxValue() {
  Data max = std::get<1>(histogram_[0]);

  for (typename std::vector<std::tuple<int, Data>>::iterator it =
           std::next(histogram_.begin());
       it != histogram_.end(); ++it) {
    if (std::get<0>(*it) > max) max = std::get<0>(*it);
  }

  return max;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline Data Histogram<Data>::GetMinValue() {
  Data min = histogram_[1][0];

  for (int i = 1; i < size_; ++i) {
    if (histogram_[1] < min) min = histogram_[1];
  }

  return min;
}

//------------------------------------------------------------------------------
//

template <typename Data>
auto Histogram<Data>::GetMinIndex() -> double {
  for (int i = 0; i < size_; ++i) {
    if (histogram_[1][i] == min_histogram_) return histogram_[0][i];
  }
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline auto Histogram<Data>::GetMaxIndex() -> double {
  for (int i = 0; i < size_; ++i) {
    if (histogram_[1][i] == max_histogram_) return histogram_[0][i];
  }
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline void Histogram<Data>::SetInterNumber(double inter) {
  inter_ = inter;
  inter_func_ = false;
}

template <typename Data>
inline void Histogram<Data>::SetInterFunction(unsigned int function) {
  pfunc_inter_ = (unsigned int (*)(unsigned int))function;
  inter_func_ = true;
}

}  // namespace