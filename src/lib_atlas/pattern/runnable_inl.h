/**
 * \file	runnable_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_PATTERN_RUNNABLE_H_
#error This file may only be included from runnable.h
#endif

namespace atlas {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE Runnable::Runnable() ATLAS_NOEXCEPT : stop_(), thread_() {}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE Runnable::~Runnable() ATLAS_NOEXCEPT { stop(); }

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void Runnable::start() ATLAS_NOEXCEPT {
  thread_ = std::thread(&Runnable::run, this);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void Runnable::stop() ATLAS_NOEXCEPT {
  stop_ = true;
  thread_.join();
}

} // namespace atlas
