// Copyright (c) 2022, The MaPra Authors.

#include <algorithm>

#include "gtest/gtest.h"

TEST(ExampleTest, Multiplication) { EXPECT_EQ(2.5 * 3.0, 7.5); }

TEST(ExampleTest, Min) { EXPECT_EQ(std::min(3.4, 2.7), 2.7); }
