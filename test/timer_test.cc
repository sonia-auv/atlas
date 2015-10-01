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

  timer.Start();
  timer.Sleep(10);
  atlas::Timer<>::Sleep(10);
  auto value1 = timer.Time();
  timer.Sleep(10);
  auto value2 = timer.Time();

  ASSERT_TRUE(timer.IsRunning());
  ASSERT_GT(value1, 0);
  ASSERT_GT(value2, value1);
}

TEST(TimerTest, pauseTimer) {
  atlas::NanoTimer timer;

  timer.Start();
  timer.Pause();
  auto value1 = timer.Time();
  auto value2 = timer.Time();

  ASSERT_FALSE(timer.IsRunning());
  ASSERT_GT(value1, 0);
  ASSERT_EQ(value1, value2);
}

TEST(TimerTest, unpauseTimer) {
  atlas::NanoTimer timer;

  timer.Start();
  timer.Pause();
  auto value1 = timer.Time();
  timer.Unpause();
  auto value2 = timer.Time();

  ASSERT_TRUE(timer.IsRunning());
  ASSERT_GT(value1, 0);
  ASSERT_GT(value2, value1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
