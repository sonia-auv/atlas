/// \file	service_client_manager.h
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \date	23/05/2015
/// \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
/// Use of this source code is governed by the MIT license that can be
/// found in the LICENSE file.

#ifndef LIB_ATLAS_ROS_SERVICE_CLIENT_MANAGER_H_
#define LIB_ATLAS_ROS_SERVICE_CLIENT_MANAGER_H_

// ROS Libraries
#include <ros/ros.h>

#include <lib_atlas/macros.h>

namespace atlas {

/// This class is an helper for storing ServiceClient.
///
/// By inheriting this class and then call the RegisterService, you abstract
/// the managment of storing and deleting ROS Services.
class ServiceClientManager {
 public:
  static constexpr unsigned short kConnectionAttempts = 3;

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ServiceClientManager(const ros::NodeHandle &hdl) ATLAS_NOEXCEPT;

  virtual ~ServiceClientManager() {
    for (auto &service : services_) {
      service.second.shutdown();
    }
  }

  //============================================================================
  // P U B L I C   M E T H O D S

  /// The method register a service given its name and a pointer to the callback
  /// method that will handle the callback.
  ///
  /// \param name  The name of the service you want to register.
  /// \param function  A pointer to the the defined callback.
  /// \param manager Take a reference to the real object in order to call
  /// ROS advertiseService.
  template <typename M>
  auto RegisterService(const std::string &service_name) -> void {
    auto result_advertise = node_handler_.serviceClient<M>(service_name);
    auto pair = std::pair<std::string, ros::ServiceClient>(service_name,
                                                           result_advertise);
    services_.insert(pair);
  }

  /// Shutdown a service given its name.
  ///
  /// \return True if the service was shutdown correctly.
  auto ShutdownService(const std::string &service_name) -> bool {
    for (auto &service : services_) {
      if (service.first == service_name) {
        service.second.shutdown();
        services_.erase(service.first);
        return true;
      }
    }
    return false;
  }

  /// Get a service given its name.
  ///
  /// \return A pointer to the service. This will return nullptr if there is no
  /// pointer with this name.
  auto GetService(const std::string &service_name)
      -> ros::ServiceClient *const {
    for (auto &service : services_) {
      if (service.first == service_name) {
        return &(service.second);
      }
    }
    return nullptr;
  }

  template <typename T>
  auto SecureCall(const T &service, std::string &node) -> bool {
    for (int i = 0; i < kConnectionAttempts; ++i) {
      if (services_.at(node).call(service)) {
        return true;
      }
    }
    return false;
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  /// The Node Handler provided by ROS to manage nodes
  ros::NodeHandle node_handler_;

  /// List of ROS services offered by this class
  std::map<std::string, ros::ServiceClient> services_;
};

}  // namespace atlas

#endif  // LIB_ATLAS_ROS_SERVICE_CLIENT_MANAGER_H_