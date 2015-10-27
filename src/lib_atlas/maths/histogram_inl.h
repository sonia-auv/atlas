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
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram(const Histogram<Tp_> &rhs)
    ATLAS_NOEXCEPT {
  inter_ = rhs.inter_;
  pfunc_inter_ = rhs.pfunc_inter_;
  histogram_ = rhs.histogram_;
  inter_func_ = rhs.inter_func_;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram(Histogram<Tp_> &&rhs)
    ATLAS_NOEXCEPT {
  inter_ = std::move(rhs.inter_);
  pfunc_inter_ = std::move(rhs.pfunc_inter_);
  histogram_ = std::move(rhs.histogram_);
  inter_func_ = std::move(rhs.inter_func_);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram(const std::vector<Tp_> &data,
                                              double inter) ATLAS_NOEXCEPT
    : inter_(inter),
      inter_func_(false),
      pfunc_inter_(nullptr),
      histogram_() {
  for (const auto &e : data) {
    Add(e);
  }
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::Histogram(std::vector<Tp_> const &data,
                                              unsigned int function)
    ATLAS_NOEXCEPT : pfunc_inter_((unsigned int (*)(unsigned int))function),
                     inter_func_(true),
                     inter_(0),
                     histogram_() {
  for (const auto &e : data) {
    Add(e);
  }
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Histogram<Tp_>::~Histogram() ATLAS_NOEXCEPT {}

//==============================================================================
// O P E P R A T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename Tp_>
Histogram<Tp_> &Histogram<Tp_>::operator=(const Histogram<Tp_> &rhs)
    ATLAS_NOEXCEPT {
  inter_ = rhs.inter_;
  pfunc_inter_ = rhs.pfunc_inter_;
  histogram_ = rhs.histogram_;
  inter_func_ = rhs.inter_func_;
  return *this;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
Histogram<Tp_> &Histogram<Tp_>::operator=(Histogram<Tp_> &&rhs) ATLAS_NOEXCEPT {
  inter_ = std::move(rhs.inter_);
  pfunc_inter_ = std::move(rhs.pfunc_inter_);
  histogram_ = std::move(rhs.histogram_);
  inter_func_ = std::move(rhs.inter_func_);
  return *this;
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE uint64_t Histogram<Tp_>::Index(const Tp_ &value)
    ATLAS_NOEXCEPT {
  return std::distance(std::begin(histogram_), histogram_.find(value));
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_
Histogram<Tp_>::At(const uint64_t &index) const ATLAS_NOEXCEPT {
  return histogram_.at(index);
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ Histogram<Tp_>::Max() const ATLAS_NOEXCEPT {
  return (std::max_element(std::begin(histogram_), std::end(histogram_)))
      ->first;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE Tp_ Histogram<Tp_>::Min() const ATLAS_NOEXCEPT {
  return (std::min_element(std::begin(histogram_), std::end(histogram_)))
      ->first;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE double Histogram<Tp_>::GetMinIndex() const ATLAS_NOEXCEPT {
  return std::distance(std::begin(histogram_), Min());
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE double Histogram<Tp_>::GetMaxIndex() const ATLAS_NOEXCEPT {
  return std::distance(std::begin(histogram_), Max());
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::SetInterNumber(double inter)
    ATLAS_NOEXCEPT {
  inter_ = inter;
  inter_func_ = false;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::SetInterFunction(unsigned int function)
    ATLAS_NOEXCEPT {
  pfunc_inter_ = (unsigned int (*)(unsigned int))function;
  inter_func_ = true;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE void Histogram<Tp_>::Add(const Tp_ &data) ATLAS_NOEXCEPT {
  if (histogram_.find(data) == histogram_.end()) {
    histogram_[data] = 1;
  } else {
    ++histogram_[data];
  }
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE std::shared_ptr<Histogram<Tp_>>
Histogram<Tp_>::ZoomOnValues(const Tp_ &begin, const Tp_ &end) ATLAS_NOEXCEPT {
  std::vector<Tp_> new_values;
  for (const auto &e : histogram_) {
    if (e.first < end && e.first >= begin) {
      new_values.push_back(e.first);
    }
  }

  auto new_histo = std::make_shared<Histogram<Tp_>>(new_values, inter_);
  new_histo->inter_func_ = inter_func_;
  new_histo->pfunc_inter_ = pfunc_inter_;

  return new_histo;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE std::shared_ptr<Histogram<Tp_>>
Histogram<Tp_>::ZoomOnIndexes(const uint64_t &begin, const uint64_t &end) {
  if (end - begin < 0) {
    throw std::invalid_argument(
        "The end index cannot be superior to the begin index");
  }

  auto begin_it = histogram_.begin();
  std::advance(begin_it, begin);
  auto first_it = begin_it;
  std::advance(begin_it, end - begin + 1);
  auto last_it = begin_it;

  auto new_histo = std::make_shared<Histogram<Tp_>>();
  for (; first_it != last_it; std::advance(first_it, 1)) {
    new_histo->Add(first_it->first);
  }

  new_histo->inter_ = inter_;
  new_histo->inter_func_ = inter_func_;
  new_histo->pfunc_inter_ = pfunc_inter_;

  return new_histo;
}

//------------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_ALWAYS_INLINE uint64_t
Histogram<Tp_>::Count(const Tp_ &value) const ATLAS_NOEXCEPT {
  try {
    return histogram_.at(value);
  } catch (const std::out_of_range &e) {
    return 0;
  }
}

}  // namespace atlas
