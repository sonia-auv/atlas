/**
 * \file	trigo_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/10/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * Use of this source code is governed by the GNU GPL license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <lib_atlas/maths/trigo.h>
#include <lib_atlas/maths/numbers.h>

using namespace atlas;

TEST(Trigo, angle_conversions) {
  double a1_deg = 0;
  double a1_rad = 0;
  double a2_deg = -572.958;
  double a2_rad = -10;
  double a3_deg = 57.2958;
  double a3_rad = 1.000000358;
  double a4_deg = 7677.63;
  double a4_rad = 134.;

  ASSERT_EQ(SetPrecision(DegToRad(a1_deg), 6), SetPrecision(a1_rad, 6));
  ASSERT_EQ(SetPrecision(RadToDeg(a1_rad), 6), SetPrecision(a1_deg, 6));

  ASSERT_EQ(SetPrecision(DegToRad(a2_deg), 6), SetPrecision(a2_rad, 6));
  ASSERT_EQ(SetPrecision(RadToDeg(a2_rad), 6), SetPrecision(a2_deg, 6));

  ASSERT_EQ(SetPrecision(DegToRad(a3_deg), 6), SetPrecision(a3_rad, 6));
  ASSERT_EQ(SetPrecision(RadToDeg(a3_rad), 6), SetPrecision(a3_deg, 6));

  ASSERT_EQ(SetPrecision(DegToRad(a4_deg), 6), SetPrecision(a4_rad, 6));
  ASSERT_EQ(SetPrecision(RadToDeg(a4_rad), 6), SetPrecision(a4_deg, 6));
}

TEST(Trigo, normalize) {
  ASSERT_EQ(SetPrecision(NormalizeAngle(45.5), 6), SetPrecision(45.5, 6));
  ASSERT_EQ(SetPrecision(NormalizeAngle(-29.), 6), SetPrecision(360.-29, 6));
  ASSERT_EQ(SetPrecision(NormalizeAngle(720.), 6), SetPrecision(0., 6));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
