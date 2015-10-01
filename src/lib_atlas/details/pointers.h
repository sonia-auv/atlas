/**
 * \file	pointers.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_UTILS_POINTERS_H_
#define LIB_ATLAS_UTILS_POINTERS_H_

#include <memory>
#include <type_traits>
#include <utility>

namespace std {

//------------------------------------------------------------------------------
//
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace std

#endif  // LIB_ATLAS_UTILS_POINTERS_H_