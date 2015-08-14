/**
 * \file service_server_manager.h
 * \author Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date 23/05/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_ROS_SERVICE_SERVER_MANAGER_H_
#define ATLAS_ROS_SERVICE_SERVER_MANAGER_H_

// ROS Libraries
#include <ros/ros.h>

// Sonia Atlas includes
#include <lib_atlas/ros/ros_manager.h>
#include <lib_atlas/details/macros.h>

namespace atlas {

/// This class is an helper for storing ServiceServer.
///
/// By inheriting this class and then call the RegisterService, you abstract
/// the managment of storing and deleting ROS Services.
template <class T>
class ServiceServerManager {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  /// A pointer to the real callback that you are going to define in your class.
  template <typename M>
  using CallBackPtr = bool (T::*)(typename M::Request &,
                                  typename M::Response &);

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ServiceServerManager(const ROSManager &manager) ATLAS_NOEXCEPT
      : node_handler_(manager.node_handler()),
        services_() {}

  virtual ~ServiceServerManager() {
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
  auto RegisterService(const std::string &name, CallBackPtr<M> function,
                       T &manager) -> void {
    if (function != nullptr) {
      auto result_advertise =
          node_handler_.advertiseService(name.c_str(), function, &manager);
      auto pair =
          std::pair<std::string, ros::ServiceServer>(name, result_advertise);
      services_.insert(pair);
    }
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
      -> ros::ServiceServer *const {
    for (auto &service : services_) {
      if (service.first == service_name) {
        return &(service.second);
      }
    }
    return nullptr;
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  /// The Node Handler provided by ROS to manage nodes
  ros::NodeHandle node_handler_;

  /// List of ROS services offered by this class
  std::map<std::string, ros::ServiceServer> services_;
};

}  // namespace atlas

#endif  // ATLAS_ROS_SERVICE_SERVER_MANAGER_H_
