//
// Created by gemini on 9/10/15.
//

#include <gtest/gtest.h>
#include <lib_atlas/maths/histogram.h>

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

TEST(HistogramTest, CreationHistogram) {
  atlas::Histogram test(v1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}