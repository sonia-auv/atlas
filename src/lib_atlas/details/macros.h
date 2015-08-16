/**
 * \file	macros.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_DETAILS_MACROS_H_
#define ATLAS_DETAILS_MACROS_H_

#if (__cplusplus >= 201103L)
#define ATLAS_NOEXCEPT noexcept
#define ATLAS_NOEXCEPT_(x) noexcept(x)
#define ATLAS_NOEXCEPT_OR_FALSE(x) noexcept(x)
#else
#define ATLAS_NOEXCEPT throw()
#define ATLAS_NOEXCEPT_(x)
#define ATLAS_NOEXCEPT_OR_FALSE(x) false
#endif

#ifndef ATLAS_ALWAYS_INLINE
#define ATLAS_ALWAYS_INLINE \
  __attribute__((__visibility__("default"), __always_inline__)) inline
#endif

#endif  // ATLAS_DETAILS_MACROS_H_
