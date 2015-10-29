/**
 * \file	image_sequence_provider_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_IO_IMAGE_SEQUENCE_CAPTURE_H_
#error This file may only be included from image_sequence_capture.h
#endif

#include <functional>
#include <lib_atlas/sys/timer.h>

namespace atlas {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImageSequenceCapture::ImageSequenceCapture() ATLAS_NOEXCEPT
    : max_framerate_(0),
      frame_count_(0),
      total_streaming_time_(0),
      streaming_(false),
      running_(false),
      streaming_thread_(),
      cv(),
      cv_mutex_() {}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImageSequenceCapture::~ImageSequenceCapture()
    ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE const cv::Mat &ImageSequenceCapture::GetImage() {
  if (!IsStreaming()) {
    ++frame_count_;
    return GetNextImage();
  }
  throw std::logic_error(
      "The image provider is streaming, cannot get next image.");
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::Start() ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> lock(cv_mutex_);
  running_ = true;
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::Stop() ATLAS_NOEXCEPT { running_ = false; }

//------------------------------------------------------------------------------
//
double ImageSequenceCapture::GetMaxFramerate() const ATLAS_NOEXCEPT {
  return max_framerate_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::SetMaxFramerate(double framerate) {
  max_framerate_ = framerate;
}

//------------------------------------------------------------------------------
//
uint64_t ImageSequenceCapture::GetFrameCount() const ATLAS_NOEXCEPT {
  return frame_count_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceCapture::SetStreamingMode(bool streaming) ATLAS_NOEXCEPT {
  streaming_ = streaming;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceCapture::IsStreaming() const ATLAS_NOEXCEPT {
  return streaming_;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceCapture::IsRunning() const ATLAS_NOEXCEPT { return running_; }

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceCapture::StreamingLoop() ATLAS_NOEXCEPT {
  std::unique_lock<std::mutex> lock(cv_mutex_);
  cv.wait(lock, [=] { return static_cast<bool>(running_); });

  MilliTimer timer;
  timer.Start();
  while (running_) {
    if (streaming_) {
      if (max_framerate_ != 0 && max_framerate_ < 1 / timer.Seconds()) {
        cv.wait(lock, [&] { return max_framerate_ < 1 / timer.Seconds(); });
      }
      timer.Reset();
      Notify(GetNextImage());
      ++frame_count_;
    }
  }
}

}  // namespace atlas
