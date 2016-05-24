/**
 * \file	configuration_parser.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	24/05/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIB_ATLAS_ROS_CONFIGURATION_PARSER_H_
#define LIB_ATLAS_ROS_CONFIGURATION_PARSER_H_

#include <lib_atlas/macros.h>
#include <ros/ros.h>
#include <string>

namespace atlas {

class ConfigurationParser {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ConfigurationParser>;
  using ConstPtr = std::shared_ptr<const ConfigurationParser>;
  using PtrList = std::vector<ConfigurationParser::Ptr>;
  using ConstPtrList = std::vector<ConfigurationParser::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ConfigurationParser(const ros::NodeHandle &nh,
                               const std::string &name_space = "") : nh_(nh),
 name_space_(name_space) {}

  virtual ~ConfigurationParser() = default;

 protected:
  //============================================================================
  // P R I V A T E   M E T H O D S

  template <typename Tp_>
  void FindParameter(const std::string &str, Tp_ &p) ATLAS_NOEXCEPT;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;

  std::string name_space_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//-----------------------------------------------------------------------------
//
template <typename Tp_>
ATLAS_INLINE void ConfigurationParser::FindParameter(const std::string &str,
                                        Tp_ &p) ATLAS_NOEXCEPT {
  if (nh_.hasParam(name_space_ + "/" + str)) {
    nh_.getParam(name_space_ + "/" + str, p);
  } else {
    ROS_WARN_STREAM("Did not find " << name_space_ << "/" << str
                        << ". Using default value instead.");
  }
}

}  // namespace atlas

#endif  // LIB_ATLAS_ROS_CONFIGURATION_PARSER_H_
