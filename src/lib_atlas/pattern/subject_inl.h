/**
 * \file	observer_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_PATTERN_SUBJECT_H_
#error This file may only be included from subject.h
#endif

#include <assert.h>
#include <algorithm>
#include <lib_atlas/pattern/observer.h>

namespace atlas {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Subject<Args_...>::Subject() ATLAS_NOEXCEPT
    : observers_(),
      observers_mutex_() {}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Subject<Args_...>::Subject(const Subject<Args_...> &rhs)
    ATLAS_NOEXCEPT : observers_(),
                     observers_mutex_() {
  for (auto &observer : rhs.observers_) {
    observer->Observe(*this);
  }
}
//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Subject<Args_...>::~Subject() ATLAS_NOEXCEPT {
  for (const auto &observer : observers_) {
    observer->OnSubjectDisconnected(*this);
  }
}

//==============================================================================
// O P E R A T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::operator=(
    const Subject<Args_...> &rhs) ATLAS_NOEXCEPT -> void {
  DetachAll();
  for (auto &observer : rhs.observers_) {
    Attach(*observer);
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::Attach(Observer<Args_...> &observer)
    -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);

  auto it = std::find(observers_.begin(), observers_.end(), &observer);
  if (it != observers_.end()) {
    throw std::invalid_argument("The element is already in the container.");
  } else {
    observers_.push_back(&observer);
  }
  observer.OnSubjectConnected(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::Detach(Observer<Args_...> &observer)
    -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  auto it = std::find(observers_.begin(), observers_.end(), &observer);
  if (it == observers_.end()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    observers_.erase(it);
  }
  observer.OnSubjectDisconnected(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::DetachNoCallback(
    Observer<Args_...> &observer) -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  auto it = std::find(observers_.begin(), observers_.end(), &observer);
  if (it == observers_.end()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    observers_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::DetachAll() ATLAS_NOEXCEPT -> void {
  for (const auto &observer : observers_) {
    Detach(*observer);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::Notify(Args_... args)
    ATLAS_NOEXCEPT -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  for (const auto &observer : observers_) {
    observer->OnSubjectNotify(*this, args...);
  }
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::ObserverCount() const ATLAS_NOEXCEPT
    -> size_t {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  return observers_.size();
}

}  // namespace atlas
