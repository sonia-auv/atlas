/**
 * \file	time_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	28/06/2015
 * \copyright Copyright (c) 2015 Thibaut Mattio. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <gtest/gtest.h>
#include <math.h>
#include <lib_atlas/sys/fsinfo.h>

TEST(FSInfo, blockSize) {
  auto block_size = atlas::block_size();

  ASSERT_GE(block_size, 512);        // A block can not be less that 512B
  ASSERT_LE(block_size, 64 * 1024);  // A block can not be less that 64kB
}

TEST(FSInfo, freeSpace) {
  auto free_space = static_cast<int>(
      round(atlas::free_physical_memory(atlas::BitUnit::MB)));

  ASSERT_GT(free_space, 0);

  // The size of the file in MB
  unsigned int size = 32;
  FILE *fp = fopen("/tmp/atlas_fsinfo_test", "w");
  ftruncate(fileno(fp), size * 1024 * 1024);
  fclose(fp);

  // Asserting that new the available space has changed
  auto new_free_space = static_cast<int>(
      round(atlas::free_physical_memory(atlas::BitUnit::MB)));

  // Removing the created file
  remove("/tmp/atlas_fsinfo_test");

  ASSERT_EQ(free_space, new_free_space + size);
}

TEST(FSInfo, availableSpace) {
  auto avail_space = static_cast<int>(
      round(atlas::available_physical_memory(atlas::BitUnit::MB)));

  ASSERT_GT(avail_space, 0);

  // The size of the file in MB
  unsigned int size = 32;
  FILE *fp = fopen("/tmp/atlas_fsinfo_test", "w");
  ftruncate(fileno(fp), size * 1024 * 1024);
  fclose(fp);

  // Asserting that new the available space has changed
  auto new_avail_space = static_cast<int>(
      round(atlas::available_physical_memory(atlas::BitUnit::MB)));

  // Removing the created file
  remove("/tmp/atlas_fsinfo_test");

  ASSERT_EQ(avail_space, new_avail_space + size);
}

TEST(FSInfo, totalSpace) {
  auto total_space = atlas::total_physical_memory();

  ASSERT_GT(total_space, 0);
}

TEST(FSInfo, percentageUsedSpace) {
  auto value = atlas::percentage_used_physical_memory();

  ASSERT_GT(value, 0);
  ASSERT_LT(value, 1);
}

TEST(FSInfo, percentageAvailableSpace) {
  auto value = atlas::percentage_available_physical_memory();

  ASSERT_GT(value, 0);
  ASSERT_LT(value, 1);
}

TEST(FSInfo, dataAccuracy) {
  // Here we want to check that the data make sense.
  // We are going to comparate the data against each other to assert there is
  // no obvious error.

  auto total = atlas::total_physical_memory();
  auto avail = atlas::available_physical_memory();
  auto free = atlas::free_physical_memory();

  ASSERT_GT(total, avail);
  ASSERT_GT(total, free);
  ASSERT_GE(free, avail);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}