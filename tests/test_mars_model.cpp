#include <iostream>

#include "mars_model/mars_model.hpp"

#include "gtest/gtest.h"

TEST(sample_test_case, sample_test)
{
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    mars::MarsModel mars_model;
    std::cout << mars_model.test_value << std::endl;

    return 0;
}