/**
 * \file	stats_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_atlas/maths/stats.h>

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

static std::vector<int> v2 = {{
                                  157,
                                  898,
                                  89,
                                  875,
                                  222,
                                  955,
                                  451,
                                  351,
                                  839,
                                  315,
                                  544,
                                  983,
                                  719,
                                  299,
                                  62,
                                  433,
                                  769,
                                  274,
                                  814,
                                  162
                              }};


TEST(StatsTest, mean) {
  ASSERT_EQ(atlas::mean(v1), 500.1);
  ASSERT_EQ(atlas::mean(v2), 510.55);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
