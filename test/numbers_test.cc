/**
 * \file	numbers_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_atlas/maths/numbers.h>

static std::vector<int> v1 = {{
                                  828,
                                  522,
                                  832,
                                  71,
                                  609,
                                  787,
                                  179,
                                  756,
                                  259,
                                  977,
                                  816,
                                  179,
                                  330,
                                  581,
                                  124,
                                  911,
                                  78,
                                  71,
                                  869,
                                  223
                              }};

TEST(StatsTest, clamp) {
  ASSERT_EQ(atlas::Clamp(1, 0, 5), 1);
  ASSERT_EQ(atlas::Clamp(-1, 0, 3), 0);
  ASSERT_EQ(atlas::Clamp(15, 0, 3), 3);

  ASSERT_EQ(atlas::Clamp(1.f, 0.f, 3.f), 1);
  ASSERT_EQ(atlas::Clamp(-1.f, 0.f, 3.f), 0);
  ASSERT_EQ(atlas::Clamp(15, 0, 3), 3);

  ASSERT_EQ(atlas::Clamp(543, v1), 543);
  ASSERT_EQ(atlas::Clamp(-143, v1), 71);
  ASSERT_EQ(atlas::Clamp(14143, v1), 977);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
