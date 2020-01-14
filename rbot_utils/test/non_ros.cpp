#include "gtest/gtest.h"

#include "algorithm.hpp"
#include "array.hpp"
#include "introspection.hpp"
#include "math.hpp"
#include "threshold.hpp"
#include "transform.hpp"

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
