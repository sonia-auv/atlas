/**
 * \file	image_sequence_provider.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/08/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */
#ifndef ATLAS_IO_IMAGE_SEQUENCE_PROVIDER_H_
#define ATLAS_IO_IMAGE_SEQUENCE_PROVIDER_H_

#include <atomic>
#include <mutex>
#include <thread>

#include <lib_atlas/pattern/observer.h>
#include <lib_atlas/sys/timer.h>
#include <opencv2/core/core.hpp>

namespace atlas {

class ImageSequenceProvider : public Subject<const cv::Mat &> {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  ImageSequenceProvider() ATLAS_NOEXCEPT;

  ~ImageSequenceProvider() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C  M E T H O D S

  /**
   * Return the max framerate. The default value for the max framerate is 0.
   *
   * \return The max framerate being used.
   */
  virtual double max_framerate() const ATLAS_NOEXCEPT;

  /**
   * Set a max framerate for the streaming mode.
   *
   * When in streaming mode. If a max framerate have been set, the streaming
   * loop will wait the appropriate time in order to have the expected max
   * framerate (only if the framerate is higher that the one manually
   * specified).
   */
  virtual void set_max_framerate(double framerate);

  /**
   * \return The total of frame count from the moment the ImageSequenceProvider
   *         have been created.
   */
  virtual uint64_t frame_count() const ATLAS_NOEXCEPT;

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
  const cv::Mat &image() const;

  /**
   * Start the ImageSequenceProvider by Openning the media -- see Open().
   */
  void start() ATLAS_NOEXCEPT;

  /**
   * Stop the ImageSequenceProvider by closing the media -- see Close().
   */
  void stop() ATLAS_NOEXCEPT;

  /**
   * Set the streaming mode to true or false.
   *
   * Synchronise the streaming flag ressource and set it to the passed argument
   *
   * \param streaming The flag to enable or disable the streaming mode.
   */
  void set_streaming(bool streaming) ATLAS_NOEXCEPT;

  /**
   * Return either if the ImageSequenceProvider is in streaming mode
   *
   * \return True if in streaming mode, False else.
   */
  bool streaming() const ATLAS_NOEXCEPT;

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  virtual const cv::Mat &GetNextImage() const = 0;

  virtual void Open() noexcept = 0;

  virtual void Close() noexcept = 0;

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  /**
   * The thread function that is going to notify all the observer of this
   * Image Provider if we are in streaming mode.
   *
   * This is going to wait for the running_ flag to be true, and the will start.
   */
  void StreamingLoop() ATLAS_NOEXCEPT;

  //============================================================================
  // P R I V A T E   M E M B E R S

  double max_framerate_ = {0};

  uint64_t frame_count_ = {0};

  double total_streaming_time_ = {0};

  std::atomic<bool> streaming_ = {false};

  std::atomic<bool> running_ = {false};

  mutable std::mutex image_mutex_ = {};

  std::unique_ptr<std::thread> streaming_thread_ = {};

  std::condition_variable cv = {};

  mutable std::mutex cv_mutex_ = {};
};

}  // namespace atlas

#include <lib_atlas/io/image_sequence_provider_inl.h>

#endif  // ATLAS_IO_IMAGE_SEQUENCE_PROVIDER_H_
