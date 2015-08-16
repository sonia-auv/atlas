/**
 * \file	typedef.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	16/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_TYPEDEF_H_
#define ATLAS_TYPEDEF_H_

#include <ros/ros.h>
#include <memory>

namespace atlas {

using NodeHandlePtr = std::shared_ptr<ros::NodeHandle>;

} // namespace atlas

#endif  // ATLAS_TYPEDEF_H_
