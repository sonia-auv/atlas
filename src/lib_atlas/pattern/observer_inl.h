/**
 * \file	observer_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_PATTERN_OBSERVER_H_
#error This file may only be included from observer.h
#endif

#include <cassert>
#include <algorithm>

namespace atlas {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Observer<Args_...>::Observer() ATLAS_NOEXCEPT
    : subjects_(),
      subjects_mutex_() {}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Observer<Args_...>::Observer(const Observer<Args_...> &rhs)
    ATLAS_NOEXCEPT : subjects_(),
                     subjects_mutex_() {
  for (auto &subject : rhs.subjects_) {
    subject->Attach(*this);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Observer<Args_...>::Observer(Subject<Args_...> &subject)
    ATLAS_NOEXCEPT : subjects_(),
                     subjects_mutex_() {
  subject.Attach(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Observer<Args_...>::~Observer() ATLAS_NOEXCEPT {
  for (const auto &subject : subjects_) {
    subject->DetachNoCallback(*this);
  }
}

//==============================================================================
// O P E R A T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE void Observer<Args_...>::operator=(
    const Observer<Args_...> &rhs) ATLAS_NOEXCEPT {
  DetachFromAllSubject();
  for (auto &subject : rhs.subjects_) {
    subject->Attach(*this);
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE void Observer<Args_...>::DetachFromAllSubject()
    ATLAS_NOEXCEPT {
  // Do not lock the mutex here because it will be in OnSubjectDisconnected().
  for (const auto &subject : subjects_) {
    subject->Detach(*this);
  }
  subjects_.clear();
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE void Observer<Args_...>::OnSubjectConnected(
    Subject<Args_...> &subject) ATLAS_NOEXCEPT {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  auto it = std::find(subjects_.begin(), subjects_.end(), &subject);
  if (it != subjects_.end()) {
    throw std::invalid_argument("The element is already in the container.");
  } else {
    subjects_.push_back(&subject);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE void Observer<Args_...>::OnSubjectDisconnected(
    Subject<Args_...> &subject) {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  auto it = std::find(subjects_.begin(), subjects_.end(), &subject);
  if (it == subjects_.end()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    subjects_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE bool Observer<Args_...>::IsAttached(
    const Subject<Args_...> &subject) const ATLAS_NOEXCEPT {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  return std::find(subjects_.begin(), subjects_.end(), &subject) !=
         subjects_.end();
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE void Observer<Args_...>::Observe(
    Subject<Args_...> &subject) {
  subject.Attach(*this);
}

}  // namespace atlas
