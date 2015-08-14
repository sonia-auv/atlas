/**
 * \file	image_sequence_provider_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_IO_IMAGE_SEQUENCE_PROVIDER_H_
#error This file may only be included from image_sequence_provider.h
#endif

#include <lib_atlas/sys/timer.h>
#include <lib_atlas/details/pointers.h>
#include <functional>

namespace atlas {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImageSequenceProvider::ImageSequenceProvider()
    ATLAS_NOEXCEPT {
  streaming_thread_ = std::unique_ptr<std::thread>(
      new std::thread(&ImageSequenceProvider::StreamingLoop, this));
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImageSequenceProvider::~ImageSequenceProvider()
    ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE const cv::Mat &ImageSequenceProvider::image() const {
  if (!streaming()) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    return GetNextImage();
  }
  throw std::logic_error(
      "The image provider is streaming, cannot get next image.");
}

//------------------------------------------------------------------------------
//
void ImageSequenceProvider::start() ATLAS_NOEXCEPT {
  std::unique_lock<std::mutex> lock(image_mutex_);
  Open();
  {
    std::lock_guard<std::mutex> lock(cv_mutex_);
    running_ = true;
  }
}

//------------------------------------------------------------------------------
//
void ImageSequenceProvider::stop() ATLAS_NOEXCEPT {
  running_ = false;
  std::lock_guard<std::mutex> lock(image_mutex_);
  Close();
}

//------------------------------------------------------------------------------
//
double ImageSequenceProvider::max_framerate() const ATLAS_NOEXCEPT {
  return max_framerate_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceProvider::set_max_framerate(double framerate) {
  max_framerate_ = framerate;
}

//------------------------------------------------------------------------------
//
uint64_t ImageSequenceProvider::frame_count() const ATLAS_NOEXCEPT {
  return frame_count_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceProvider::set_streaming(bool streaming) ATLAS_NOEXCEPT {
  streaming_ = streaming;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceProvider::streaming() const ATLAS_NOEXCEPT {
  return streaming_;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceProvider::StreamingLoop() ATLAS_NOEXCEPT {
  std::unique_lock<std::mutex> lock(cv_mutex_);
  cv.wait(lock, [=] { return static_cast<bool>(running_); });

  MilliTimer timer;
  timer.start();
  while (running_) {
    if (streaming_) {
      if (max_framerate_ != 0 && max_framerate_ < 1 / timer.seconds()) {
        cv.wait(lock, [=] { return max_framerate_ < 1 / timer.seconds(); });
      }
      timer.reset();

      std::unique_lock<std::mutex> lock(image_mutex_);
      Notify(GetNextImage());
    }
  }
}

}  // namespace atlas
