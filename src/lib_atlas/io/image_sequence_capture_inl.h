/**
 * \file	image_sequence_provider_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_IO_IMAGE_SEQUENCE_CAPTURE_H_
#error This file may only be included from image_sequence_capture.h
#endif

#include <lib_atlas/sys/timer.h>
#include <lib_atlas/details/pointers.h>
#include <functional>

namespace atlas {

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE const cv::Mat &ImageSequenceCapture::image() {
  if (!streaming()) {
    ++frame_count_;
    return GetNextImage();

  }
  throw std::logic_error(
      "The image provider is streaming, cannot get next image.");
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::start() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> lock(cv_mutex_);
  running_ = true;
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::stop() ATLAS_NOEXCEPT {
  running_ = false;
}

//------------------------------------------------------------------------------
//
double ImageSequenceCapture::max_framerate() const ATLAS_NOEXCEPT {
  return max_framerate_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::set_max_framerate(double framerate) {
  max_framerate_ = framerate;
}

//------------------------------------------------------------------------------
//
uint64_t ImageSequenceCapture::frame_count() const ATLAS_NOEXCEPT {
  return frame_count_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::set_streaming(bool streaming) ATLAS_NOEXCEPT {
  streaming_ = streaming;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceCapture::streaming() const ATLAS_NOEXCEPT {
  return streaming_;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceCapture::running() const ATLAS_NOEXCEPT {
  return running_;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceCapture::StreamingLoop() ATLAS_NOEXCEPT {
  std::unique_lock<std::mutex> lock(cv_mutex_);
  cv.wait(lock, [=] { return static_cast<bool>(running_); });

  MilliTimer timer;
  timer.start();
  while (running_) {
    if (streaming_) {
      if (max_framerate_ != 0 && max_framerate_ < 1 / timer.seconds()) {
        cv.wait(lock, [&] { return max_framerate_ < 1 / timer.seconds(); });
      }
      timer.reset();
      Notify(GetNextImage());
      ++frame_count_;
    }
  }
}

}  // namespace atlas
