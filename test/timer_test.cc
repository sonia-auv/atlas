/**
 * \file	timer_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_atlas/sys/timer.h>

TEST(TimerTest, startTimer) {
  atlas::NanoTimer timer;

  timer.start();
  timer.sleep(10);
  atlas::Timer<>::sleep(10);
  auto value1 = timer.time();
  timer.sleep(10);
  auto value2 = timer.time();

  ASSERT_TRUE(timer.is_running());
  ASSERT_GT(value1, 0);
  ASSERT_GT(value2, value1);
}

TEST(TimerTest, pauseTimer) {
  atlas::NanoTimer timer;

  timer.start();
  timer.pause();
  auto value1 = timer.time();
  auto value2 = timer.time();

  ASSERT_FALSE(timer.is_running());
  ASSERT_GT(value1, 0);
  ASSERT_EQ(value1, value2);
}

TEST(TimerTest, unpauseTimer) {
  atlas::NanoTimer timer;

  timer.start();
  timer.pause();
  auto value1 = timer.time();
  timer.unpause();
  auto value2 = timer.time();

  ASSERT_TRUE(timer.is_running());
  ASSERT_GT(value1, 0);
  ASSERT_GT(value2, value1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
