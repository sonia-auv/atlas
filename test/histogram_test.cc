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
  test.CreateHistogram();
  ASSERT_EQ(test.GetMaxValue(), 5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}