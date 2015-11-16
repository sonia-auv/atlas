/**
 * \file	io_exception.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	16/11/2015
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

#ifndef LIB_ATLAS_EXCEPTIONS_IO_EXCEPTION_H_
#define LIB_ATLAS_EXCEPTIONS_IO_EXCEPTION_H_

#include <string>
#include <iostream>
#include <stdexcept>
#include <lib_atlas/macros.h>

namespace atlas {

class IOException : public std::exception {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  explicit IOException(std::string file, int line, int errnum)
      : file_(file), line_(line), errno_(errnum) {
    std::stringstream ss;
    char *error_str = strerror(errnum);
    ss << "IO Exception (" << errno_ << "): " << error_str;
    ss << ", file " << file_ << ", line " << line_ << ".";
    e_what_ = ss.str();
  }

  explicit IOException(std::string file, int line, const char *description)
      : file_(file), line_(line), errno_(0) {
    std::stringstream ss;
    ss << "IO Exception: " << description;
    ss << ", file " << file_ << ", line " << line_ << ".";
    e_what_ = ss.str();
  }

  explicit IOException(const IOException &other)
      : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

  virtual ~IOException() ATLAS_NOEXCEPT {}

  //============================================================================
  // P U B L I C   O P E R A T O R S

  IOException &operator=(const IOException &) = delete;

  //============================================================================
  // P U B L I C   M E T H O D S

  int GetErrorNumber() { return errno_; }

  const char *what() const ATLAS_NOEXCEPT override { return e_what_.c_str(); }

 private:
  //============================================================================
  // P R I V A T E   M E M B E R S

  std::string file_;

  int line_;

  std::string e_what_;

  int errno_;
};

}  // namespace atlas

#endif  // LIB_ATLAS_EXCEPTIONS_IO_EXCEPTION_H_
