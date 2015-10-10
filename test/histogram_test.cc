//
// Created by gemini on 9/10/15.
//

#include <gtest/gtest.h>
#include <lib_atlas/maths/histogram.h>

static std::vector<double> v1 = {{
                                     2.0,
                                     2.0,
                                     5.0,
                                     1.0,
                                     9.0,
                                     7.0,
                                     9.0,
                                     3.0,
                                     5.0,
                                     9.0,
                                     1.0,
                                     9.0,
                                     3.0,
                                     1.0,
                                     2.0,
                                     11.0,
                                     0.0,
                                     1.0,
                                     9.0,
                                     2.0
                                 }};

TEST(HistogramTest, CreationHistogram) {
  atlas::Histogram<double> test(v1,1.0);
  ASSERT_EQ(test.GetMaxValue(), 5);
  ASSERT_EQ(test.GetMinValue(), 0);
  ASSERT_EQ(test.GetMaxIndex(), 11);
  ASSERT_EQ(test.GetMinIndex(), 0);
/*
  atlas::Histogram<double> test2(v1,3.0);
  ASSERT_EQ(test.GetMaxValue(), 5);
  ASSERT_EQ(test.GetMinValue(), 0);
  ASSERT_EQ(test.GetMaxIndex(), 11);
  ASSERT_EQ(test.GetMinIndex(), 0);
  */
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}