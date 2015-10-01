/**
 * \file	image_sequence_provider.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_IO_IMAGE_SEQUENCE_WRITER_H_
#define LIB_ATLAS_IO_IMAGE_SEQUENCE_WRITER_H_

#include <atomic>
#include <mutex>
#include <thread>
#include <opencv2/core/core.hpp>
#include <lib_atlas/macros.h>
#include <lib_atlas/sys/timer.h>
#include <lib_atlas/pattern/observer.h>

namespace atlas {

class ImageSequenceWriter : public Observer<const cv::Mat &> {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  ImageSequenceWriter() ATLAS_NOEXCEPT;

  virtual ~ImageSequenceWriter() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C  M E T H O D S

  /**
   * \return The total of frame count from the moment the ImageSequenceProvider
   *         have been created.
   */
  virtual uint64_t FrameCount() const ATLAS_NOEXCEPT;

  /**
   * If the ImageSequenceProvider is not streaming, this will return the next
   * image of the sequence.
   *
   * If the ImageSequenceProvider is currently streaming, the image will rather
   * be sent to all the observer and the users should not try to get the image
   * manually.
   *
   * \return The next image, if the ImageSequenceProvider is not streaming.
   */
  void Write(const cv::Mat &);

  /**
   * Start the ImageSequenceProvider by Openning the media -- see Open().
   */
  void Start() ATLAS_NOEXCEPT;

  /**
   * Stop the ImageSequenceProvider by closing the media -- see Close().
   */
  void Stop() ATLAS_NOEXCEPT;

  /**
   * Returns either if the ImageSequence is running or not.
   *
   * You can set the running state of the ImageSequence by calling start or stop
   * methods.
   *
   * \return The running state of the ImageSequence
   */
  bool IsRunning() const ATLAS_NOEXCEPT;

  /**
   * Set the streaming mode to true or false.
   *
   * Synchronise the streaming flag ressource and set it to the passed argument
   *
   * \param streaming The flag to enable or disable the streaming mode.
   */
  void SetStreamingMode(bool streaming) ATLAS_NOEXCEPT;

  /**
   * Return either if the ImageSequenceProvider is in streaming mode
   *
   * \return True if in streaming mode, False else.
   */
  bool IsStreaming() const ATLAS_NOEXCEPT;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  virtual void WriteImage(const cv::Mat &image) = 0;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * The thread function that is going to notify all the observer of this
   * Image Provider if we are in streaming mode.
   *
   * This is going to wait for the running_ flag to be true, and the will start.
   */
  virtual void OnSubjectNotify(Subject<const cv::Mat &> &subject,
                               const cv::Mat &image) ATLAS_NOEXCEPT override;

  //============================================================================
  // P R I V A T E   M E M B E R S

  uint64_t frame_count_;

  bool streaming_;

  bool running_;
};

}  // namespace atlas

#include <lib_atlas/io/image_sequence_writer_inl.h>

#endif  // LIB_ATLAS_IO_IMAGE_SEQUENCE_WRITER_H_
