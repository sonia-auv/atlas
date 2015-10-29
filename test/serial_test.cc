#include <string>
#include "gtest/gtest.h"
#include <boost/bind.hpp>
#include <lib_atlas/io/serial.h>

#if defined(OS_LINUX)
#include <pty.h>
#else
#include <util.h>
#endif

using namespace atlas;

namespace {

class SerialTests : public ::testing::Test {
protected:
  virtual void SetUp() {
    if (openpty(&master_fd, &slave_fd, name, NULL, NULL) == -1) {
      perror("openpty");
      exit(127);
    }

    ASSERT_TRUE(master_fd > 0);
    ASSERT_TRUE(slave_fd > 0);
    ASSERT_TRUE(std::string(name).length() > 0);

    port1 = new Serial(std::string(name), 115200, Timeout::simpleTimeout(250));
  }

  virtual void TearDown() {
    port1->Close();
    delete port1;
  }

  Serial * port1;
  int master_fd;
  int slave_fd;
  char name[100];
};

TEST_F(SerialTests, readWorks) {
  write(master_fd, "abc\n", 4);
  std::string r = port1->Read(4);
  EXPECT_EQ(r, std::string("abc\n"));
}

TEST_F(SerialTests, writeWorks) {
  char buf[5] = "";
  port1->Write("abc\n");
  read(master_fd, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("abc\n"));
}

TEST_F(SerialTests, timeoutWorks) {
  // Timeout a read, returns an empty string
  std::string empty = port1->Read();
  EXPECT_EQ(empty, std::string(""));
  
  // Ensure that writing/reading still works after a timeout.
  write(master_fd, "abc\n", 4);
  std::string r = port1->Read(4);
  EXPECT_EQ(r, std::string("abc\n"));
}

TEST_F(SerialTests, partialRead) {
  // Write some data, but request more than was written.
  write(master_fd, "abc\n", 4);

  // Should timeout, but return what was in the buffer.
  std::string empty = port1->Read(10);
  EXPECT_EQ(empty, std::string("abc\n"));
  
  // Ensure that writing/reading still works after a timeout.
  write(master_fd, "abc\n", 4);
  std::string r = port1->Read(4);
  EXPECT_EQ(r, std::string("abc\n"));
}

}  // namespace

int main(int argc, char **argv) {
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}
