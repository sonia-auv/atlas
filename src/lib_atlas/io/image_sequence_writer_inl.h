/**
 * \file	image_sequence_provider_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_IO_IMAGE_SEQUENCE_WRITER_H_
#error This file may only be included from image_sequence_writer.h
#endif

#include <lib_atlas/sys/timer.h>
#include <lib_atlas/details/pointers.h>
#include <functional>

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
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::OnSubjectNotify(
    Subject<const cv::Mat &> &subject, const cv::Mat &image) ATLAS_NOEXCEPT
    -> void {
  if (streaming()) {
    WriteImage(image);
    ++frame_count_;
    return;
  }
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::write(const cv::Mat &image)
    -> void {
  if (running_) {
    if (!streaming()) {
      ++frame_count_;
      WriteImage(image);
      return;
    }
    throw std::logic_error(
        "The image writer is not running, cannot write the image.");
  }
  throw std::logic_error(
      "The image writer is streaming, cannot write the image.");
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::start() ATLAS_NOEXCEPT -> void {
  running_ = true;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::stop() ATLAS_NOEXCEPT -> void {
  running_ = false;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::frame_count() const ATLAS_NOEXCEPT
    -> uint64_t {
  return frame_count_;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::set_streaming(bool streaming)
    ATLAS_NOEXCEPT -> void {
  streaming_ = streaming;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::streaming() const ATLAS_NOEXCEPT
    -> bool {
  return streaming_;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE auto ImageSequenceWriter::running() const ATLAS_NOEXCEPT
    -> bool {
  return running_;
}

}  // namespace atlas
