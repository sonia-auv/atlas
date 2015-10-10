/**
 * \file	histogram_inl.h
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	8/09/2015
 * \copyright Copyright (c) 2015 Antoine Dozois. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MATHS_HISTOGRAM_H_
#error This file may only be included histogram.h
#endif  // LIB_ATLAS_HISTOGRAM_H_

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
    : min_data_(Min(data)),
      max_data_(Max(data)),
      inter_(inter),
      inter_func_(false),
      pfunc_inter_(nullptr),
      data_(data),
      histogram_(),
      size_(max_data_ - min_data_) {
  CreateHistogram();
}

//------------------------------------------------------------------------------
//

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::Histogram(std::vector<Data> const &data,
                                               unsigned int function)
    : min_data_(Min(data)),
      max_data_(Max(data)),
      min_histogram_(0),
      max_histogram_(0),
      pfunc_inter_((unsigned int (*)(unsigned int))function),
      inter_func_(true),
      inter_(0),
      data_(data),
      histogram_(),
      size_(max_data_ - min_data_) {
  CreateHistogram();
}

template <typename Data>
ATLAS_ALWAYS_INLINE Histogram<Data>::~Histogram() {}
//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//

template <typename Data>
inline void Histogram<Data>::CreateHistogram() {
  // std::sort(data_.begin(), data_.end());
  std::map<Data, int> histo_init_ = {{min_data_, 0}};
  typename std::map<Data, int>::iterator it_histo;

  for (int i = 1; i <= floor(max_data_ / inter_); i++) {
    histo_init_.insert(std::make_pair<Data, int>(
        static_cast<Data>(min_data_ + (i * inter_)), 0));
  }
  for (auto const &it : data_) {
    it_histo = histo_init_.find(it);
    it_histo->second += 1;
  }

  /*
  int i = 0;
  int add = 0;
  for (typename std::vector<Data>::iterator it = std::next(data_.begin());
       it != (data_.end() + 1); it++) {
    std::map<Data, int>::iterator it_map = histo_init_.begin();
    if (*it - it_map->first <= inter_) {
      it_map->second+=1;
    } else if (*it - it_map->first > inter_) {
      add = floor((*it - it_map->first)/inter_);
      for (int j = 0; j < add; j++) {
        histo_init_.insert(std::make_pair<Data,int >(data_[0]+,0));
      }

      i++;
    } else {
      for (int j = std::get<1>(histo_init_[i]); j < *it; j++) {
        histo_init_.push_back(std::make_tuple(0, j + 1));
      }
      i += (*it - std::get<1>(histo_init_[i]));
      std::get<0>(histo_init_[i]) += 1;
    }
    */

  histogram_ = histo_init_;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline Data Histogram<Data>::GetMaxValue() {
  Data max = histogram_.begin()->second;
  for (typename std::map<Data, int>::iterator it =
           std::next(histogram_.begin());
       it != histogram_.end(); ++it) {
    if (it->second > max) {
      max = it->second;
    }
  }
  return max;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline Data Histogram<Data>::GetMinValue() {
  Data min = histogram_.begin()->second;
  for (typename std::map<Data, int>::iterator it =
           std::next(histogram_.begin());
       it != histogram_.end(); ++it) {
    if (it->second < min) {
      min = it->second;
    }
  }
  return min;
}

//------------------------------------------------------------------------------
//

template <typename Data>
auto Histogram<Data>::GetMinIndex() -> double {
  Data min = histogram_.begin()->first;
  return min;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline auto Histogram<Data>::GetMaxIndex() -> double {
  Data max = histogram_.rbegin()->first;
  return max;
}

//------------------------------------------------------------------------------
//

template <typename Data>
inline void Histogram<Data>::SetInterNumber(double inter) {
  inter_ = inter;
  inter_func_ = false;
  CreateHistogram();
}

template <typename Data>
inline void Histogram<Data>::SetInterFunction(unsigned int function) {
  pfunc_inter_ = (unsigned int (*)(unsigned int))function;
  inter_func_ = true;
  CreateHistogram();
}

}  // namespace atlas