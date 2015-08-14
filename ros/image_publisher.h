/// \file	image_publisher.h
/// \author	Thibaut Mattio <thibaut.mattio@gmail.com>
/// \date	23/05/2015
/// \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
/// Use of this source code is governed by the MIT license that can be
/// found in the LICENSE file.

#ifndef ATLAS_ROS_IMAGE_PUBLISHER_H_
#define ATLAS_ROS_IMAGE_PUBLISHER_H_

#include <mutex>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <lib_atlas/ros/ros_manager.h>
#include <lib_atlas/details/macros.h>

namespace atlas {

class ImagePublisher {
 public:
  //============================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ImagePublisher(const ROSManager &manager,
                          const std::string &topic_name)
      : topic_name_(manager.node_name() + topic_name),
        img_transport_(manager.node_handler()),
        publisher_(img_transport_.advertise(topic_name_, 1)),
        topic_mutex_() {}

  virtual ~ImagePublisher() { publisher_.shutdown(); }

  //============================================================================
  // P U B L I C   M E T H O D S

  /// Function to call to publish an image on a topic.
  auto PublishImage(const cv::Mat &image) -> void {
    topic_mutex_.lock();
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    publisher_.publish(msg);
    topic_mutex_.unlock();
  }

  void PublishVideo(const std::string &video_file_path, double start_time,
                    double stop_time);

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  const std::string topic_name_;

  image_transport::ImageTransport img_transport_;

  image_transport::Publisher publisher_;

  mutable std::mutex topic_mutex_;
};

}  // namespace atlas

#endif  // ATLAS_ROS_IMAGE_PUBLISHER_H_
