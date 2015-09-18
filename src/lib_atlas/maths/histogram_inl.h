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
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data)
    : min_data_(min(data)),
      max_data_(max(data)),
      inter_((max_data_ - min_data_) / data.size()),
      data_(data),
      histogram_(NULL),
      size_(ceil((max_data_ - min_data_) / inter_)),
      max_histogram_(GetMaxValue()),
      min_histogram_(GetMinValue()),
      value_histogram_(0) {}

//------------------------------------------------------------------------------
//

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data,
                                         double inter)
    : min_data_(min(data)),
      max_data_(max(data)),
      inter_(inter),
      data_(data),
      histogram_(NULL),
      size_(ceil((max_data_ - min_data_) / inter_)),
      max_histogram_(GetMaxValue()),
      min_histogram_(GetMinValue()),
      value_histogram_(0) {}

//------------------------------------------------------------------------------
//

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data,
                                         double max, double min)
    : min_data_(min),
      max_data_(max),
      inter_((max_data_ - min_data_) / data.size()),
      data_(data),
      histogram_(NULL),
      size_(ceil((max_data_ - min_data_) / inter_)),
      max_histogram_(GetMaxValue()),
      min_histogram_(GetMinValue()),
      value_histogram_(0) {}

//------------------------------------------------------------------------------
//

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data,
                                         double inter, double max, double min)
    : min_data_(min),
      max_data_(max),
      inter_(inter),
      data_(data),
      histogram_(NULL),
      size_(ceil((max_data_ - min_data_) / inter_)),
      max_histogram_(GetMaxValue()),
      min_histogram_(GetMinValue()),
      value_histogram_(0) {}

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::~Histogram();
//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//

template <typename Data>
inline std::vector<Data>* Histogram<Data>::CreateHistogram() {
  std::vector<std::vector<Data> > histo_init_(size_);

  for (int i = 0; i < size_; ++i) {
    histo_init_[0][i] = min_data_ + (inter_ * (i + 1));
  }

  for (int j = 0; j < data_.size(); ++j) {
    histo_init_[1][(data_[j] - min_data_) / inter_] += 1;
  }

  histogram_ = histo_init_;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline Data Histogram<Data>::GetMaxValue() {
  double max = histogram_[1][0];

  for (int i = 1; i < size_; ++i) {
    if (histogram_[1][i] > max) max = histogram_[1][i];
  }

  return max;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline Data Histogram<Data>::GetMinValue() {
  double min = histogram_[1][0];
  ;

  for (int i = 1; i < size_; ++i) {
    if (histogram_[1][i] < min) min = histogram_[1][i];
  }

  return min;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline auto Histogram<Data>::GetMinIndex() {
  for (int i = 0; i < size_; ++i) {
    if (histogram_[1][i] == min_histogram_) return histogram_[0][i];
  }
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline auto Histogram<Data>::GetMaxIndex() {
  for (int i = 0; i < size_; ++i) {
    if (histogram_[1][i] == max_histogram_) return histogram_[0][i];
  }
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline void Histogram<Data>::SetInter(double inter) { inter_ = inter; }

}  // namespace