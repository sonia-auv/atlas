/**
 * \file	thread_pool.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef ATLAS_PATTERN_THREAD_POOL_H_
#define ATLAS_PATTERN_THREAD_POOL_H_

#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

#include <lib_atlas/macros.h>

namespace atlas {

//==============================================================================
// C L A S S E S

/**
 * The ThreadPool class provides an
 */
class ThreadPool {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ThreadPool(size_t) ATLAS_NOEXCEPT;

  ~ThreadPool() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C  M E T H O D S

  template <class Tp_, class... Args_>
  auto Enqueue(Tp_ &&f, Args_ &&... args)
      -> std::future<typename std::result_of<Tp_(Args_...)>::type>;

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  std::vector<std::thread> workers_ = {};

  std::queue<std::function<void()>> tasks_ = {};

  mutable std::mutex queue_mutex_ = {};

  std::condition_variable condition_ = {};

  bool is_stoped_ = {false};
};

}  // namespace atlas

#endif  // ATLAS_PATTERN_THREAD_POOL_H_
