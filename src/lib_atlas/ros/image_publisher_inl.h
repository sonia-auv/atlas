/**
 * \file	image_publisher_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_ROS_IMAGE_PUBLISHER_H_
#error This file may only be included from image_publisher.h
#endif

#include <cassert>
#include <algorithm>

namespace atlas {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImagePublisher::ImagePublisher(
    const std::string &topic_name) ATLAS_NOEXCEPT
    : topic_name_(topic_name),
      img_transport_(ros::NodeHandle()),
      publisher_() {
  publisher_ = img_transport_.advertise(topic_name_, 1);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImagePublisher::~ImagePublisher() ATLAS_NOEXCEPT {
  publisher_.shutdown();
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImagePublisher::WriteImage(const cv::Mat &image)
    ATLAS_NOEXCEPT {
  if (!image.empty()) {
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher_.publish(msg);
    cv::waitKey(1);
  }
}

}  // namespace atlas
