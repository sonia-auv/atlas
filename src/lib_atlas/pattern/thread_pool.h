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

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ThreadPool::ThreadPool(size_t threads) ATLAS_NOEXCEPT {
  for (size_t i = 0; i < threads; ++i) {
    workers_.emplace_back([this] {
      for (;;) {
        std::function<void()> task;

        {
          auto lock = std::unique_lock<std::mutex>{queue_mutex_};
          condition_.wait(lock,
                          [this] { return is_stoped_ || !tasks_.empty(); });
          if (is_stoped_ && tasks_.empty()) {
            return;
          }
          task = std::move(tasks_.front());
          tasks_.pop();
        }

        task();
      }
    });
  }
}

//------------------------------------------------------------------------------
//
ThreadPool::~ThreadPool() ATLAS_NOEXCEPT {
  {
    auto lock = std::unique_lock<std::mutex>{queue_mutex_};
    is_stoped_ = true;
  }
  condition_.notify_all();
  for (std::thread &worker : workers_) {
    worker.join();
  }
}

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_, class... Args_>
auto ThreadPool::Enqueue(Tp_ &&f, Args_ &&... args)
    -> std::future<typename std::result_of<Tp_(Args_...)>::type> {
  using return_type = typename std::result_of<Tp_(Args_...)>::type;

  auto task = std::make_shared<std::packaged_task<return_type()>>(
      std::bind(std::forward<Tp_>(f), std::forward<Args_>(args)...));

  std::future<return_type> res = task->get_future();

  {
    auto lock = std::unique_lock<std::mutex>{queue_mutex_};

    // don't allow enqueueing after stopping the pool
    if (is_stoped_) {
      throw std::runtime_error("enqueue on stopped ThreadPool");
    }

    tasks_.emplace([task]() { (*task)(); });
  }

  condition_.notify_one();
  return res;
}

}  // namespace atlas

#endif  // ATLAS_PATTERN_THREAD_POOL_H_
