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
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::OnSubjectNotify(
    Subject<const cv::Mat &> &subject, const cv::Mat &image) ATLAS_NOEXCEPT {
  if (streaming()) {
    WriteImage(image);
    ++frame_count_;
    return;
  }
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE void ImageSequenceWriter::write(const cv::Mat &image) {
  if (running_ && !streaming()) {
    ++frame_count_;
    WriteImage(image);
    return;
  }
  throw std::logic_error(
      "The image capture is streaming, cannot write the image.");
}

//------------------------------------------------------------------------------
//
void ImageSequenceWriter::start() ATLAS_NOEXCEPT { running_ = true; }

//------------------------------------------------------------------------------
//
void ImageSequenceWriter::stop() ATLAS_NOEXCEPT { running_ = false; }

//------------------------------------------------------------------------------
//
uint64_t ImageSequenceWriter::frame_count() const ATLAS_NOEXCEPT {
  return frame_count_;
}

//------------------------------------------------------------------------------
//
void ImageSequenceWriter::set_streaming(bool streaming) ATLAS_NOEXCEPT {
  streaming_ = streaming;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceWriter::streaming() const ATLAS_NOEXCEPT {
  return streaming_;
}

//------------------------------------------------------------------------------
//
bool ImageSequenceWriter::running() const ATLAS_NOEXCEPT { return running_; }

}  // namespace atlas
