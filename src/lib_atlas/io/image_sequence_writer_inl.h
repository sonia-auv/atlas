/**
 * \file	image_sequence_provider_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_IO_IMAGE_SEQUENCE_WRITER_H_
#error This file may only be included from image_sequence_writer.h
#endif

#include <functional>
#include <lib_atlas/sys/timer.h>

namespace atlas {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImageSequenceWriter::ImageSequenceWriter() ATLAS_NOEXCEPT
    : frame_count_(0),
      streaming_(false),
      running_(false) {}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE ImageSequenceWriter::~ImageSequenceWriter() ATLAS_NOEXCEPT {
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::OnSubjectNotify(
    Subject<const cv::Mat &> &subject, const cv::Mat &image) ATLAS_NOEXCEPT {
  if (IsStreaming()) {
    WriteImage(image);
    ++frame_count_;
    return;
  }
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::Write(const cv::Mat &image) {
  if (running_) {
    if (!IsStreaming()) {
      ++frame_count_;
      WriteImage(image);
      return;
    }
    throw std::logic_error(
        "The image writer is not running, cannot Write the image.");
  }
  throw std::logic_error(
      "The image writer is streaming, cannot Write the image.");
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::Start() ATLAS_NOEXCEPT {
  running_ = true;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::Stop() ATLAS_NOEXCEPT {
  running_ = false;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE uint64_t
ImageSequenceWriter::FrameCount() const ATLAS_NOEXCEPT {
  return frame_count_;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::SetStreamingMode(bool streaming)
    ATLAS_NOEXCEPT {
  streaming_ = streaming;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE bool ImageSequenceWriter::IsStreaming() const
    ATLAS_NOEXCEPT {
  return streaming_;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE bool ImageSequenceWriter::IsRunning() const ATLAS_NOEXCEPT {
  return running_;
}

}  // namespace atlas
