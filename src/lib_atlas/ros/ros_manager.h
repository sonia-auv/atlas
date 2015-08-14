/// \file	ros_manager.h
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \date	24/05/2015
/// \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
/// Use of this source code is governed by the MIT license that can be
/// found in the LICENSE file.

#ifndef ATLAS_ROS_ROS_MANAGER_H_
#define ATLAS_ROS_ROS_MANAGER_H_

#include <string>
#include <ros/ros.h>

// Sonia Atlas includes
#include <lib_atlas/details/pointers.h>

namespace atlas {

class ROSManager {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ROSManager(int argc, char **argv, const std::string &node_name)
      : node_name_(node_name) {
    ros::init(argc, argv, node_name);
    node_handler_ = std::make_shared<ros::NodeHandle>();
  }

  ~ROSManager() = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  auto node_handler() const -> const ros::NodeHandle & {
    return *node_handler_;
  }

  auto node_name() const -> const std::string & { return node_name_; }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  /// The Node Handler provided by ROS to manage nodes
  std::shared_ptr<ros::NodeHandle> node_handler_;

  std::string node_name_;
};

}  // namespace atlas

#endif  // ATLAS_ROS_ROS_MANAGER_H_
