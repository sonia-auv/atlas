/**
 * \file	timer_inl.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_SYSTEM_TIMER_H_
#error This file may only be included from timer.h
#endif

#include <math.h>
#include <thread>

namespace atlas {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE Timer<Up_, Tp_>::Timer() ATLAS_NOEXCEPT {}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE Timer<Up_, Tp_>::~Timer() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::start() ATLAS_NOEXCEPT -> void {
  reset();
  std::lock_guard<std::mutex> guard(member_guard_);
  is_running_ = true;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::reset() ATLAS_NOEXCEPT -> void {
  std::lock_guard<std::mutex> guard(member_guard_);
  start_time_ = Tp_::now();
  pause_time_ = start_time_;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::pause() -> void {
  std::lock_guard<std::mutex> guard(member_guard_);
  if (!is_running_) {
    guard.~lock_guard();
    throw std::logic_error("The timer is not running");
  }
  pause_time_ = Tp_::now();
  is_running_ = false;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::unpause() -> void {
  std::lock_guard<std::mutex> guard(member_guard_);
  if (is_running_) {
    guard.~lock_guard();
    throw std::logic_error("The timer is running");
  }
  start_time_ += Tp_::now() - pause_time_;
  is_running_ = true;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::is_running() ATLAS_NOEXCEPT -> bool {
  std::lock_guard<std::mutex> guard(member_guard_);
  return is_running_;
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
template <typename Yp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::time() const ATLAS_NOEXCEPT
    -> double {
  std::lock_guard<std::mutex> guard(member_guard_);
  auto time = Tp_::now() - start_time_;
  if (!is_running_) {
    time = pause_time_ - start_time_;
  }
  auto period = static_cast<double>(Yp_::period::num) /
                static_cast<double>(Yp_::period::den);
  return static_cast<double>(std::chrono::duration_cast<Yp_>(time).count() *
                             period);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::now() ATLAS_NOEXCEPT -> int64_t {
  return std::chrono::duration_cast<Up_>(Tp_::now().time_since_epoch()).count();
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::sleep(int64_t sleeping_time)
    ATLAS_NOEXCEPT -> void {
  std::this_thread::sleep_for(Up_(sleeping_time));
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::nanoseconds() const ATLAS_NOEXCEPT
    -> int64_t {
  return static_cast<int64_t>(time<std::chrono::nanoseconds>() * 1000000000);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::microseconds() const ATLAS_NOEXCEPT
    -> int64_t {
  return static_cast<int64_t>(time<std::chrono::microseconds>() * 1000000);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::milliseconds() const ATLAS_NOEXCEPT
    -> int64_t {
  return static_cast<int64_t>(time<std::chrono::milliseconds>() * 1000);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::seconds() const ATLAS_NOEXCEPT
    -> int64_t {
  return static_cast<int64_t>(time<std::chrono::seconds>());
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::minutes() const ATLAS_NOEXCEPT
    -> int64_t {
  return static_cast<int64_t>(time<std::chrono::minutes>() / 60);
}

//------------------------------------------------------------------------------
//
template <class Up_, class Tp_>
ATLAS_ALWAYS_INLINE auto Timer<Up_, Tp_>::hours() const ATLAS_NOEXCEPT
    -> int64_t {
  return static_cast<int64_t>(time<std::chrono::hours>() / 3600);
}

}  // namespace atlas
