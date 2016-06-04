/**
 * \file	matrix_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	21/02/2016
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
#include <lib_atlas/maths/matrix.h>

TEST(MatrixTest, QuatToEuler) {
  Eigen::Quaterniond q;
  q.w() = 0.00501813488735;
  q.x() = -0.0822811981613;
  q.y() = -0.996591510943;
  q.z() = 0.00316064590555;

  auto xyz = atlas::QuatToEuler(q);
  ASSERT_NEAR(xyz(0), -3.134466730930046, 0.001);
  ASSERT_NEAR(xyz(1), -0.009482079820205, 0.001);
  ASSERT_NEAR(xyz(2), 2.976807314666800, 0.001);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
