//
// Created by gemini on 9/10/15.
//

#include <gtest/gtest.h>
#include <lib_atlas/maths/averaging.h>

static std::vector<double> v1 = {{
                                  828.0,
                                  522.0,
                                  832.0,
                                  71.0,
                                  609.0,
                                  787.0,
                                  179.0,
                                  756.0,
                                  259.0,
                                  977.0,
                                  816.0,
                                  179.0,
                                  330.0,
                                  581.0,
                                  124.0,
                                  911.0,
                                  78.0,
                                  71.0,
                                  869.0,
                                  223.0
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


TEST(StatsTest, Mean) {
  ASSERT_EQ(atlas::Mean(v1), 500.1);
  ASSERT_EQ(atlas::Mean(v2), 510.55);
}

TEST(StatsTest, Median) {
  ASSERT_EQ(atlas::Median(v1), 581);
  ASSERT_EQ(atlas::Median(v2), 451);
}


TEST(StatsTest, GeometricMean) {
  // Added ceil function for averaging the number with 3 decimals precision
  ASSERT_EQ(ceil(atlas::GeometricMean(v1)*1000)/1000, 361.123);
  ASSERT_EQ(ceil(atlas::GeometricMean(v2)*1000)/1000, 394.537);
}



TEST(StatsTest, HarmonicMean) {
  //  Added ceil function for averaging the number with 3 decimals precision
  ASSERT_EQ(ceil(atlas::HarmonicMean(v1)*1000)/1000, 231.529);
  ASSERT_EQ(ceil(atlas::HarmonicMean(v2)*1000)/1000, 273.124);
}



int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}