/**
 * \file	observer_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_PATTERN_OBSERVER_H_
#error This file may only be included from observer.h
#endif

#include <cassert>
#include <algorithm>

namespace atlas {

namespace details {

//------------------------------------------------------------------------------
//
template <class Tp_, class Ut_>
ATLAS_ALWAYS_INLINE auto PushUniqueRef(std::vector<Ut_> &ctnr,
                                       Tp_ &ref) -> void {
  auto it = std::find(ctnr.cbegin(), ctnr.cend(), &ref);
  if (it != ctnr.cend()) {
    throw std::invalid_argument("The element is already in the container.");
  } else {
    ctnr.emplace_back(Ut_{&ref});
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_, class Ut_>
ATLAS_ALWAYS_INLINE auto EraseRef(std::vector<Ut_> &ctnr,
                                  const Tp_ &ref) -> void {
  auto it = std::find(ctnr.cbegin(), ctnr.cend(), &ref);
  if (it == ctnr.cend()) {
    throw std::invalid_argument("The element is not in the container.");
  } else {
    ctnr.erase(it);
  }
}

}  // namespace details

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Observer<Args_...>::Observer() ATLAS_NOEXCEPT
    : subjects_{} {}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Observer<Args_...>::Observer(Subject<Args_...> &subject)
    ATLAS_NOEXCEPT {
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

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE Subject<Args_...>::Subject() ATLAS_NOEXCEPT : observers_{} {
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
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::Attach(Observer<Args_...> &observer)
    -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  details::PushUniqueRef(observers_, observer);
  observer.OnSubjectConnected(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::Detach(Observer<Args_...> &observer)
    -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  details::EraseRef(observers_, observer);
  observer.OnSubjectDisconnected(*this);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Subject<Args_...>::DetachNoCallback(
    Observer<Args_...> &observer) -> void {
  std::unique_lock<std::mutex> locker(observers_mutex_);
  details::EraseRef(observers_, observer);
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

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Observer<Args_...>::DetachFromAllSubject()
    ATLAS_NOEXCEPT -> void {
  // Do not lock the mutex here because it will be in OnSubjectDisconnected().
  for (const auto &subject : subjects_) {
    subject->Detach(*this);
  }
  subjects_.clear();
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Observer<Args_...>::OnSubjectConnected(
    Subject<Args_...> &subject) ATLAS_NOEXCEPT -> void {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  details::PushUniqueRef(subjects_, subject);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Observer<Args_...>::OnSubjectDisconnected(
    Subject<Args_...> &subject) -> void {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  details::EraseRef(subjects_, subject);
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Observer<Args_...>::IsAttached(
    const Subject<Args_...> &subject) const ATLAS_NOEXCEPT -> bool {
  std::unique_lock<std::mutex> locker(subjects_mutex_);
  return std::find(subjects_.cbegin(), subjects_.cend(), &subject) !=
         subjects_.cend();
}

//------------------------------------------------------------------------------
//
template <typename... Args_>
ATLAS_ALWAYS_INLINE auto Observer<Args_...>::Observe(Subject<Args_...> &subject)
    -> void {
  subject.Attach(*this);
}

}  // namespace atlas
