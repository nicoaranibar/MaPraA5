// Copyright (c) 2022, The MaPra Authors.

#include "gtest/gtest.h"

// Link against this file to create a binary which runs a test.
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
