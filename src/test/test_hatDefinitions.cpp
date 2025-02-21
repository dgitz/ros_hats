/*! \file test_hatDefinitions.cpp
 */
#include <gtest/gtest.h>
#include <ros_hats/HatDefinition.h>
#include <stdio.h>
using namespace ros_hats;
TEST(BasicTest, TestDefinitions) {
    for (uint8_t i = 0; i <= (uint8_t)HatDefinition::HatType::END_OF_LIST; ++i) {
        if ((i == (uint8_t)HatDefinition::HatType::UNKNOWN) ||
            (i == (uint8_t)HatDefinition::HatType::END_OF_LIST)) {
            std::string str = HatDefinition::HatTypeString((HatDefinition::HatType)i);
            EXPECT_EQ(str, "UNKNOWN");
            HatDefinition::HatType type = HatDefinition::HatTypeEnum(str);
            EXPECT_EQ(type, HatDefinition::HatType::UNKNOWN);
        }
        else {
            std::string str = HatDefinition::HatTypeString((HatDefinition::HatType)i);
            HatDefinition::HatType type = HatDefinition::HatTypeEnum(str);
            EXPECT_EQ(type, (HatDefinition::HatType)i);
        }
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}