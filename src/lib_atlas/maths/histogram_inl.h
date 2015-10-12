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
#include <iterator>

namespace atlas {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram()
    : min_data_(0),
      max_data_(0),
      inter_(0),
      inter_func_(false),
      pfunc_inter_(nullptr),
      histogram_() {}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram(std::vector<Tp_> const &data,
                                              double inter)
    : min_data_(Min(data)),
      max_data_(Max(data)),
      inter_(inter),
      inter_func_(false),
      pfunc_inter_(nullptr),
      histogram_() {
  CreateHistogram(data);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram(std::vector<Tp_> const &data,
                                              unsigned int function)
    : min_data_(Min(data)),
      max_data_(Max(data)),
      pfunc_inter_((unsigned int (*)(unsigned int))function),
      inter_func_(true),
      inter_(0),
      histogram_() {
  CreateHistogram(data);
}

template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::~Histogram() {}

//==============================================================================
// O P E P R A T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename Tp_>
Histogram<Tp_> Histogram<Tp_>::operator=(const Histogram<Tp_> &histo) {
  Histogram<Tp_> copy_histo;
  this->inter_ = histo.inter_;
  this->pfunc_inter_ = histo.pfunc_inter_;
  this->histogram_ = histo.histogram_;
  this->inter_func_ = histo.inter_func_;
  this->max_data_ = histo.max_data_;
  this->min_data_ = histo.min_data_;

  return copy_histo;
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::CreateHistogram(
    std::vector<Tp_> const &data) {
  std::map<Tp_, int> histo_init_ = {{min_data_, 0}};
  typename std::map<Tp_, int>::iterator it_histo = histo_init_.begin();
  for (int i = 1; i <= floor(max_data_ / inter_); ++i) {
    histo_init_.insert(std::make_pair<Tp_, int>(
        static_cast<Tp_>(min_data_ + (i * inter_)), 0));
  }
  int index;
  for (auto const &it : data) {
    index = floor((it - min_data_) / inter_);
    std::advance(it_histo, index);
    it_histo->second += 1;
    it_histo = histo_init_.begin();
  }

  histogram_ = histo_init_;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ Histogram<Tp_>::GetMaxValue() {
  Tp_ max = histogram_.begin()->second;
  for (typename std::map<Tp_, int>::iterator it = std::next(histogram_.begin());
       it != histogram_.end(); ++it) {
    if (it->second > max) {
      max = it->second;
    }
  }
  return max;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ Histogram<Tp_>::GetMinValue() {
  Tp_ min = histogram_.begin()->second;
  for (typename std::map<Tp_, int>::iterator it = std::next(histogram_.begin());
       it != histogram_.end(); ++it) {
    if (it->second < min) {
      min = it->second;
    }
  }
  return min;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE double Histogram<Tp_>::GetMinIndex() const ATLAS_NOEXCEPT {
  Tp_ min = histogram_.begin()->first;
  return min;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE double Histogram<Tp_>::GetMaxIndex() const ATLAS_NOEXCEPT {
  Tp_ max = histogram_.rbegin()->first;
  return max;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::SetInterNumber(double inter) {
  inter_ = inter;
  inter_func_ = false;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::SetInterFunction(
    unsigned int function) {
  pfunc_inter_ = (unsigned int (*)(unsigned int))function;
  inter_func_ = true;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::AddData(std::vector<Tp_> const &data) {
  Tp_ max_from_data = Max(data);
  Tp_ min_from_data = Min(data);

  if (min_data_ > min_from_data) {
    int add = ceil((min_data_ - min_from_data) / inter_);
    for (int i = add - 1; i >= 0; --i) {
      histogram_.insert(
          histogram_.begin(),
          std::make_pair(static_cast<Tp_>(min_from_data + (inter_ * i)), 0));
    }
  }

  if (max_data_ < max_from_data) {
    int add = ceil((max_from_data - max_data_) / inter_);
    for (int i = 1; i <= add; ++i) {
      histogram_.insert(
          std::make_pair(static_cast<Tp_>(max_data_ + (inter_ * i)), 0));
    }
  }

  int index;
  typename std::map<Tp_, int>::iterator it_histo;
  for (auto const &it : data) {
    index = ceil((it - min_data_) / inter_);
    it_histo = histogram_.find(it);
    it_histo->second += 1;
  }
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_> Histogram<Tp_>::ZoomHistogram(Tp_ begin_zoom,
                                                                 Tp_ end_zoom) {
  Histogram<Tp_> new_histo;
  for (typename std::map<Tp_, int>::iterator it =
           histogram_.find(ceil((begin_zoom - min_data_) / inter_));
       it != histogram_.find(ceil((end_zoom - min_data_) / inter_) + 1); ++it) {
    new_histo.histogram_.insert(std::make_pair(it->first, it->second));
  }

  new_histo.inter_ = this->inter_;
  new_histo.max_data_ = new_histo.histogram_.rbegin()->first;
  new_histo.min_data_ = new_histo.histogram_.begin()->first;
  new_histo.inter_func_ = this->inter_func_;
  new_histo.pfunc_inter_ = this->pfunc_inter_;

  return new_histo;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE int Histogram<Tp_>::FindOccurencie(Tp_ value) {
  int index = ceil((value - min_data_) / inter_);
  typename std::map<Tp_, int>::iterator it = histogram_.begin();
  std::advance(it, index);
  return it->second;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::RefactorHistogram() {
  std::map<Tp_, int> refactor_histo = {
      {histogram_.begin()->first, histogram_.begin()->second}};
  typename std::map<Tp_, int>::iterator it_ref = refactor_histo.begin();
  for (auto const &it : histogram_) {
    if (it.first + inter_ >= std::next(it).first) {
      it_ref->second += it.second;
    } else {
      refactor_histo.insert(static_cast<Tp_>(it_ref->first + inter_),
                            it.second);
    }
  }
}

}  // namespace atlas
