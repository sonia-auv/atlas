/**
 * \file	system_info_inl.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef LIB_ATLAS_SYSTEM_SYSTEM_INFO_H_
#error This file may only be included from fsinfo.h
#endif

#include <exception>
#include <stdexcept>
#include <string>
#include <fstream>
#include <math.h>
#include <sys/statvfs.h>

namespace atlas {

namespace details {

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE struct statvfs GenerateVFS(const char *path) {
  struct statvfs vfs;

  if (statvfs(path, &vfs) < 0) {
    throw std::runtime_error("Could not read system information");
  }
  return vfs;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double ConvertToBit(fsblkcnt_t block_ctn,
                                        uint64_t block_size,
                                        BitUnit unit) ATLAS_NOEXCEPT {
  auto bytes = static_cast<double>(block_ctn * block_size);
  switch (unit) {
    case BitUnit::BLOCK:
      return static_cast<double>(block_ctn);
    case BitUnit::B:
      return bytes;
    case BitUnit::KB:
      return bytes / 1024;
    case BitUnit::MB:
      return bytes / pow(1024, 2);
    case BitUnit::GB:
      return bytes / pow(1024, 3);
    case BitUnit::TB:
      return bytes / pow(1024, 4);
  }
}

}  // namespace details

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double TotalPhysicalMemory(BitUnit unit, const char *path)
    ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return details::ConvertToBit(vfs.f_blocks, vfs.f_frsize, unit);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double FreePhysicalMemory(BitUnit unit,
                                              const char *path) ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return details::ConvertToBit(vfs.f_bfree, vfs.f_frsize, unit);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double AvailablePhysicalMemory(
    BitUnit unit, const char *path) ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return details::ConvertToBit(vfs.f_bavail, vfs.f_frsize, unit);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double UsedPhysicalMemory(BitUnit unit,
                                              const char *path) ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return details::ConvertToBit(vfs.f_blocks - vfs.f_bavail, vfs.f_frsize, unit);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double PercentageUsedPhysicalMemory(const char *path)
    ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return static_cast<double>(vfs.f_blocks - vfs.f_bfree) /
         static_cast<double>(vfs.f_blocks - vfs.f_bfree + vfs.f_bavail);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE double PercentageAvailablePhysicalMemory(const char *path)
    ATLAS_NOEXCEPT {
  return 1. - PercentageUsedPhysicalMemory(path);
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE uint64_t BlockSize(const char *path) ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return vfs.f_frsize;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE uint64_t MaxFileName(const char *path) ATLAS_NOEXCEPT {
  auto vfs = details::GenerateVFS(path);
  return vfs.f_namemax;
}

//------------------------------------------------------------------------------
//
ATLAS_ALWAYS_INLINE bool FileExists(const std::string &file_path)
    ATLAS_NOEXCEPT {
  std::ifstream f(file_path);
  return f.good();
}

}  // namespace atlas
