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

TEST(StatsTest, min) {
  ASSERT_EQ(atlas::min(v1), 71);
  ASSERT_EQ(atlas::min(v2), 62);
}

TEST(StatsTest, max) {
  ASSERT_EQ(atlas::max(v1), 977);
  ASSERT_EQ(atlas::max(v2), 983);
}

TEST(StatsTest, euclidean) {
  // Expecting 2165.58
  ASSERT_EQ(floor(atlas::euclidean(v1, v2)*100), 216558);
}

TEST(StatsTest, covariance) {
  // Expecting -18828.6
  ASSERT_EQ(floor(atlas::covariance(v1, v2)*10), -188286);
}

TEST(StatsTest, std_dev) {
  // Expecting 330.425
  ASSERT_EQ(floor(atlas::std_dev(v1)*1000), 330425);
  // Expecting 316.030
  ASSERT_EQ(floor(atlas::std_dev(v2)*1000), 316030);
}

TEST(StatsTest, pearson) {
  // Expecting -0.180308
  ASSERT_EQ(floor(atlas::pearson(v1, v2)*1000000), -180309);

  std::vector<double> v11 = {.0, .1, .2, .3};
  std::vector<int> v12 = {0, 1, 2, 3};
  std::vector<double> v13 = {0, -1, -2, -3};

  ASSERT_EQ(floor(atlas::pearson(v11, v11)), 1);
  ASSERT_EQ(floor(atlas::pearson(v11, v12)), 1);
  ASSERT_EQ(floor(atlas::pearson(v11, v13)), -1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
