/**
 * \file	runnable.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_PATTERN_RUNNABLE_H_
#define ATLAS_PATTERN_RUNNABLE_H_

#include <atomic>
#include <thread>
#include <lib_atlas/macros.h>

namespace atlas {

class Runnable
{
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  Runnable() ATLAS_NOEXCEPT;

  virtual ~Runnable() ATLAS_NOEXCEPT;

  Runnable(Runnable const&) = delete;

  Runnable& operator =(Runnable const&) = delete;

  //============================================================================
  // P U B L I C  M E T H O D S

  void start() ATLAS_NOEXCEPT;

  void stop() ATLAS_NOEXCEPT;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  virtual void run() = 0;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  std::thread _thread;

  std::atomic<bool> _stop;
};

}  // namespace atlas

#include <lib_atlas/pattern/runnable_inl.h>

#endif  // ATLAS_PATTERN_RUNNABLE_H_
