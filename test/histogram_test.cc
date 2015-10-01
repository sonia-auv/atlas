/**
 * \file	histogram_test.cc
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

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
  atlas::Histogram<double> test(v1, 1.0);
  ASSERT_EQ(test.Max(), 11);
  ASSERT_EQ(test.Min(), 0);
  ASSERT_EQ(test.Index(test.Max()), 7);
  ASSERT_EQ(test.Index(test.Min()), 0);

  auto test2(test.ZoomOnValues(4, 9));
  ASSERT_EQ(test2->Max(), 7);
  ASSERT_EQ(test2->Min(), 5);
  ASSERT_EQ(test2->Index(test2->Max()), 1);
  ASSERT_EQ(test2->Index(test2->Min()), 0);
  ASSERT_EQ(test2->Count(4), 0);
  ASSERT_EQ(test2->Count(7), 1);


  atlas::Histogram<double> test3(v1, 3.0);
  ASSERT_EQ(test3.Max(), 11);
  ASSERT_EQ(test3.Min(), 0);
  ASSERT_EQ(test3.Index(test3.Max()), 7);
  ASSERT_EQ(test3.Index(test3.Min()), 0);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}