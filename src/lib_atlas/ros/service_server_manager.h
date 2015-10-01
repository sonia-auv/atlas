/**
 * \file service_server_manager.h
 * \author Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date 23/05/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_ROS_SERVICE_SERVER_MANAGER_H_
#define LIB_ATLAS_ROS_SERVICE_SERVER_MANAGER_H_

#include <assert.h>
#include <exception>
#include <algorithm>
#include <functional>
#include <map>
#include <ros/ros.h>
#include <lib_atlas/macros.h>

namespace atlas {

/**
 * This class is an helper for storing ServiceServer.
 *
 * By inheriting this class and then call the RegisterService, you abstract
 * the managment of storing and deleting ROS Services.
 */
template <class T>
class ServiceServerManager {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  /**
   * A pointer to the real callback that you are going to define in your class.
   *
   * ROS is using boost functions for delegates.
   * We don't want to rollback because of ROS code base, we will rather
   * use a row pointer to sore the function and send it to ros.
   */
  template <typename M>
  using CallBackPtr = bool (T::*)(typename M::Request &,
                                  typename M::Response &);

  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ServiceServerManager() ATLAS_NOEXCEPT : node_handler_(),
                                                   services_() {}

  virtual ~ServiceServerManager() {
    for (auto &service : services_) {
      service.second.shutdown();
    }
  }

  //============================================================================
  // P U B L I C   M E T H O D S

  /**
   * The method register a service given its name and a pointer to the callback
   * method that will handle the callback.
   *
   * \param name  The name of the service you want to register.
   * \param function  A pointer to the the defined callback.
   * \param manager Take a reference to the real object in order to call
   * ROS advertiseService.
   */
  template <typename M>
  void RegisterService(const std::string &name, CallBackPtr<M> function,
                       T &manager) {
    if (function != nullptr) {
      auto it = std::find_if(
          services_.begin(), services_.end(),
          [name](const std::pair<std::string, ros::ServiceServer> &srv)
              -> bool { return srv.first == name; });

      if (it == services_.end()) {
        auto result_advertise =
            node_handler_.advertiseService(name.c_str(), function, &manager);
        auto pair =
            std::pair<std::string, ros::ServiceServer>(name, result_advertise);
        services_.insert(pair);
        return;
      }
      throw std::invalid_argument(
          "A service with this name has already been registered.");
    }
  }

  /**
   * Shutdown a service given its name.
   *
   * \return True if the service was shutdown correctly.
   */
  void ShutdownService(const std::string &service_name) {
    auto it = std::find_if(
        services_.begin(), services_.end(),
        [service_name](const std::pair<std::string, ros::ServiceServer> &srv)
            -> bool { return srv.first == service_name; });
    if (it != services_.end()) {
      services_.erase(it);
      return;
    }
    throw std::invalid_argument("No service with such a name.");
  }

  /**
   * Get a service given its name.
   *
   * \return A pointer to the service. This will return nullptr if there is no
   * pointer with this name.
   */
  const ros::ServiceServer &GetService(const std::string &service_name) {
    auto it = std::find_if(
        services_.begin(), services_.end(),
        [service_name](const std::pair<std::string, ros::ServiceServer> &srv)
            -> bool { return srv.first == service_name; });
    if (it != services_.end()) {
      return (*it).second;
    }
    throw std::invalid_argument("No service with such a name.");
  }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  /**
   * The Node Handler provided by ROS to manage nodes.
   */
  ros::NodeHandle node_handler_;

  /**
   * List of ROS services offered by this class.
   */
  std::map<std::string, ros::ServiceServer> services_;
};

}  // namespace atlas

#endif  // LIB_ATLAS_ROS_SERVICE_SERVER_MANAGER_H_
