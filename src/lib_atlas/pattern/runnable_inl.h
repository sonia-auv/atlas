/**
 * \file	runnable_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_PATTERN_RUNNABLE_H_
#error This file may only be included from runnable.h
#endif

#include <lib_atlas/details/pointers.h>

namespace atlas {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE Runnable::Runnable() ATLAS_NOEXCEPT : stop_(), thread_() {}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE Runnable::~Runnable() ATLAS_NOEXCEPT { Stop(); }

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto Runnable::Start() -> void {
  if (thread_ == nullptr) {
    thread_ = std::make_unique<std::thread>(&Runnable::Run, this);
  } else {
    throw std::logic_error("The thread must be stoped before it is started.");
  }
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto Runnable::Stop() ATLAS_NOEXCEPT -> void {
  if (IsRunning()) {
    stop_ = true;
    thread_->join();
    thread_ = nullptr;
  } else {
    throw std::logic_error("The thread is not running.");
  }
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto Runnable::IsRunning() const ATLAS_NOEXCEPT -> bool {
  return thread_ != nullptr && thread_->joinable() && !MustStop();
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto Runnable::MustStop() const ATLAS_NOEXCEPT -> bool {
  return stop_;
}

}  // namespace atlas
