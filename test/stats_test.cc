/**
 * \file	stats_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Antoine Dozois <dozois.a@gmail.com>
 * \date	28/06/2015
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
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
  ASSERT_EQ(atlas::Mean(v1), 500.1);
  ASSERT_EQ(atlas::Mean(v2), 510.55);
}

TEST(StatsTest, median) {
  ASSERT_EQ(atlas::Median(v1), 581);
  ASSERT_EQ(atlas::Median(v2), 451);
}

TEST(StatsTest, geometric_mean) {
  // Added ceil function for averaging the number with 3 decimals precision
  ASSERT_EQ(ceil(atlas::GeometricMean(v1)*1000)/1000, 361.123);
  ASSERT_EQ(ceil(atlas::GeometricMean(v2)*1000)/1000, 394.537);
}

TEST(StatsTest, harmonic_mean) {
  //  Added ceil function for averaging the number with 3 decimals precision
  ASSERT_EQ(ceil(atlas::HarmonicMean(v1)*1000)/1000, 231.529);
  ASSERT_EQ(ceil(atlas::HarmonicMean(v2)*1000)/1000, 273.124);
}

TEST(StatsTest, min) {
  ASSERT_EQ(atlas::Min(v1), 71);
  ASSERT_EQ(atlas::Min(v2), 62);
}

TEST(StatsTest, max) {
  ASSERT_EQ(atlas::Max(v1), 977);
  ASSERT_EQ(atlas::Max(v2), 983);
}

TEST(StatsTest, euclidean) {
  // Expecting 2165.58
  ASSERT_EQ(floor(atlas::Euclidean(v1, v2)*100), 216558);
}

TEST(StatsTest, covariance) {
  // Expecting -18828.6
  ASSERT_EQ(floor(atlas::Covariance(v1, v2)*10), -188286);
}

TEST(StatsTest, std_dev) {
  // Expecting 330.425
  ASSERT_EQ(floor(atlas::StdDeviation(v1)*1000), 330425);
  // Expecting 316.030
  ASSERT_EQ(floor(atlas::StdDeviation(v2)*1000), 316030);
}

TEST(StatsTest, pearson) {
  // Expecting -0.180308
  ASSERT_EQ(floor(atlas::Pearson(v1, v2)*1000000), -180309);

  std::vector<double> v11 = {.0, .1, .2, .3};
  std::vector<int> v12 = {0, 1, 2, 3};
  std::vector<double> v13 = {0, -1, -2, -3};

  ASSERT_EQ(floor(atlas::Pearson(v11, v11)), 1);
  ASSERT_EQ(floor(atlas::Pearson(v11, v12)), 1);
  ASSERT_EQ(floor(atlas::Pearson(v11, v13)), -1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
