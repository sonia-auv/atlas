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

static std::vector<double> v2 = {{
                                     2.0,
                                     2.0,
                                     2.0,
                                     2.0,
                                     2.0,
                                     2.0,
                                     2.0,
                                     2.0,
                                    -2.0,
                                     15.0
                                 }};

TEST(HistogramTest, CreationHistogram) {
  atlas::Histogram<double> test(v1,1.0);
  ASSERT_EQ(test.GetMaxValue(), 5);
  ASSERT_EQ(test.GetMinValue(), 0);
  ASSERT_EQ(test.GetMaxIndex(), 11);
  ASSERT_EQ(test.GetMinIndex(), 0);

  test.AddData(v2);
  ASSERT_EQ(test.GetMaxValue(), 12);
  ASSERT_EQ(test.GetMinValue(), 0);
  ASSERT_EQ(test.GetMaxIndex(), 15);
  ASSERT_EQ(test.GetMinIndex(), -2);

  atlas::Histogram<double> test2;
  test2=test.ZoomHistogram(4,9);
  ASSERT_EQ(test2.GetMaxValue(), 5);
  ASSERT_EQ(test2.GetMinValue(), 0);
  ASSERT_EQ(test2.GetMaxIndex(), 9);
  ASSERT_EQ(test2.GetMinIndex(), 4);
  ASSERT_EQ(test2.FindOccurencie(4), 0);
  ASSERT_EQ(test2.FindOccurencie(7), 1);

/*
  atlas::Histogram<double> test3(v1,3.0);
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