/**
 * \file	macros.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_MACROS_H_
#define LIB_ATLAS_MACROS_H_

// Defining exception macros
#if (__cplusplus >= 201103L)
#define ATLAS_NOEXCEPT noexcept
#define ATLAS_NOEXCEPT_(x) noexcept(x)
#define ATLAS_NOEXCEPT_OR_FALSE(x) noexcept(x)
#else
#define ATLAS_NOEXCEPT throw()
#define ATLAS_NOEXCEPT_(x)
#define ATLAS_NOEXCEPT_OR_FALSE(x) false
#endif

// Defining inline macros
#ifndef ATLAS_ALWAYS_INLINE
#define ATLAS_ALWAYS_INLINE \
  __attribute__((__visibility__("default"), __always_inline__)) inline
#endif

#ifndef ATLAS_INLINE
#define ATLAS_INLINE inline
#endif

// Defining OS variables
#if defined(_WIN32)
#define OS_WINDOWS 1
#elif defined(__APPLE__)
#define OS_DARWIN 1
#elif defined(__linux__)
#define OS_LINUX 1
#endif

#endif  // LIB_ATLAS_MACROS_H_
