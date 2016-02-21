/**
 * \file	matrix_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	20/02/2016
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

#include <stdexcept>
#ifndef LIB_ATLAS_MATHS_MATRIX_H_
#error This file may only be included matrix.h
#endif  // LIB_ATLAS_MATHS_MATRIX_H_

namespace atlas {

//------------------------------------------------------------------------------
//
ATLAS_INLINE Eigen::Quaterniond RotToQuat(const Eigen::Matrix3d &m)
    ATLAS_NOEXCEPT {
    Eigen::Matrix3d eye;
    eye.setIdentity();

    Eigen::Matrix3d r = m + (eye - m*m.transpose()) * 0.5 * m;
    auto m1 = 1+ r(0,0) +  r(1,1) + r(2,2);
    auto m2 = 1+ r(0,0) -  r(1,1) - r(2,2);
    auto m3 = 1- r(0,0) +  r(1,1) - r(2,2);
    auto m4 = 1- r(0,0) -  r(1,1) + r(2,2);

    Eigen::Quaterniond b;

    if((m1 > m2) && (m1 > m3) && (m1 > m4)) {
        if(m1 > 0) {
            b.w() = (0.5*std::sqrt(m1));
            b.x() = (r(2,1)-r(1,2))/(4*b.w());
            b.y() = (r(0,2)-r(202))/(4*b.w());
            b.z() = (r(1,0)-r(0,1))/(4*b.w());
            b.normalize();
        } else {
            throw std::runtime_error("M1 won under 0.");
        }
    } else if((m2 > m3) && (m2 > m4)) {
        if(m2 > 0) {
            b.x() = 0.5 * std::sqrt(m2);
            b.y() = (r(1,0)+r(0,1))/(4*b.x());
            b.z() = (r(2, 0) + r(0, 2))/(4*b.x());
            b.w() = (r(2,1) - r(1, 2))/(4*b.x());
            b.normalize();
        } else {
            throw std::runtime_error("M2 won under 0.");
        }
    } else if (m3 > m4) {
        if (m3 > 0) {
            b.y() = 0.5*std::sqrt(m3);
            b.x() = (r(0,1) + r(1,0))/(4*b.y());
            b.z() = (r(2,1) + r(1,2))/(4*b.y());
            b.w() = (r(0,2) - r(2,0))/(4*b.y());
            b.normalize();
        } else {
            throw std::runtime_error("M3 won under 0.");
        }
    } else {
        if (m4 > 0) {
            b.z() = 0.5*std::sqrt(m4);
            b.x() = (r(0,2) + r(2,0))/(4*b.z());
            b.y() = (r(1,2) + r(2,1))/(4*b.z());
            b.w() = (r(1,0) + r(0,1))/(4*b.z());
            b.normalize();
        } else {
            throw std::runtime_error("M4 won under 0.");
        }
    }
    return b;
}

}  // namespace atlas
