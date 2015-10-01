/**
 * \file	image_publisher.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	23/05/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_ROS_IMAGE_SUBSCRIBER_H_
#define LIB_ATLAS_ROS_IMAGE_SUBSCRIBER_H_

#include <mutex>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <lib_atlas/io/image_sequence_capture.h>
#include <lib_atlas/macros.h>

namespace atlas {

class ImageSubscriber : public ImageSequenceCapture {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ImageSubscriber(const std::string &topic_name)
      : topic_name_(topic_name),
        img_transport_(ros::NodeHandle()),
        subscriber_(img_transport_.subscribe(
            topic_name_, 1, &ImageSubscriber::ImageCallback, this)),
        image_(),
        topic_mutex_() {}

  virtual ~ImageSubscriber() = default;

  //============================================================================
  // P U B L I C   M E T H O D S

  ATLAS_ALWAYS_INLINE const cv::Mat &GetImage() const { return image_; }

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
      auto cv_image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      image_ = cv_image_ptr->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
    }
  }

  //============================================================================
  // P R I V A T E   M E M B E R S

  const std::string topic_name_;

  image_transport::ImageTransport img_transport_;

  image_transport::Subscriber subscriber_;

  cv::Mat image_;

  mutable std::mutex topic_mutex_;
};

}  // namespace atlas

#endif  // LIB_ATLAS_ROS_IMAGE_SUBSCRIBER_H_
