/**
 * \file	time.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_SYSTEM_TIMER_H_
#define LIB_ATLAS_SYSTEM_TIMER_H_

#include <iostream>
#include <chrono>
#include <mutex>
#include <lib_atlas/macros.h>

namespace atlas {

template <class Ut_ = std::chrono::milliseconds,
          class Tp_ = std::chrono::steady_clock>
class Timer {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Timer() ATLAS_NOEXCEPT;

  ~Timer() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   S T A T I C   M E T H O D S

  /**
   * This method return the current count of the CPU.
   * The metadata can eventually be compared with another count of the CPU.
   *
   * \return the current number of count from the CPU.
   */
  static auto Now() ATLAS_NOEXCEPT -> int64_t;

  /**
   * Make a pause on the current calling thread.
   *
   * \param sleeping_time The time to sleep the current thread with the unit Ut_
   */
  static auto Sleep(int64_t sleeping_time) ATLAS_NOEXCEPT -> void;

  /**
   * Return the current date and time with the current format.
   *
   * The return format is YYYY-MM-DD.HH:mm:ss
   *
   * \return The current date and time as a string.
   */
  static std::string CurrentDate() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * Start the timer
   *
   * If the timer was running, this will reset the timer before restarting it.
   */
  void Start() ATLAS_NOEXCEPT;

  /**
   * Pause the timer if it is not running.
   * This will throw a std::logic_error exception if the timer is not running.
   */
  void Pause();

  /**
   * Unpause a paused timer.
   * This will throw a std::logic_error exception if the timer is running.
   */
  void Unpause();

  /**
   * Reset the timer by setting both the start and the pause time to now.
   */
  void Reset() ATLAS_NOEXCEPT;

  /**
   * \return Either if the timer is running or being paused.
   */
  bool IsRunning() ATLAS_NOEXCEPT;

  /**
   * Get the difference between now and the starting time with the give unit.
   * The unit must be compatible with std::chrono units
   * -- e.g. std::chrono::seconds.
   *
   * \tparam Tp_ The unit the result will be output with.
   * \return the elapsed time from the starting point to now.
   */
  template <class Yp_ = Ut_>
  double Time() const ATLAS_NOEXCEPT;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * nanoseconds.
   *
   * \return The elapsed time in nanoseconds.
   */
  int64_t NanoSeconds() const ATLAS_NOEXCEPT;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * microseconds.
   *
   * \return The elapsed time in microseconds.
   */
  int64_t MicroSeconds() const ATLAS_NOEXCEPT;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * milliseconds.
   *
   * \return The elapsed time in milliseconds.
   */
  int64_t MilliSeconds() const ATLAS_NOEXCEPT;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * seconds.
   *
   * \return The elapsed time in seconds.
   */
  int64_t Seconds() const ATLAS_NOEXCEPT;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * minutes.
   *
   * \return The elapsed time in minutes.
   */
  int64_t Minutes() const ATLAS_NOEXCEPT;

  /**
   * Wrapper to the running_time function that return the elapsed time in
   * hours.
   *
   * \return The elapsed time in hours.
   */
  int64_t Hours() const ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  bool is_running_ = {false};

  typename Tp_::time_point start_time_ = {};

  typename Tp_::time_point pause_time_ = {};

  mutable std::mutex member_guard_ = {};
};

using SecTimer = Timer<std::chrono::seconds, std::chrono::steady_clock>;
using MilliTimer =
    Timer<std::chrono::milliseconds, std::chrono::steady_clock>;
using MicroTimer =
    Timer<std::chrono::microseconds, std::chrono::steady_clock>;
using NanoTimer =
    Timer<std::chrono::nanoseconds, std::chrono::steady_clock>;

}  // namespace atlas

#include <lib_atlas/sys/timer_inl.h>

#endif  // LIB_ATLAS_SYSTEM_TIMER_H_
