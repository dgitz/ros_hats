/*! \file test_definitions.cpp
 */
#include <gtest/gtest.h>
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
TEST(BasicTest, TestDefintions) {
    // Test Class: HatDefinition
    {
        // Test Type: HatType
        for (uint8_t i = 1; i < (uint8_t)(HatDefinition::HatType::END_OF_LIST); ++i) {
            EXPECT_FALSE(HatDefinition::HatTypeString((HatDefinition::HatType)(i)) == "UNKNOWN");
        }
    }

    // Test Class: ChannelDefinition
    {
        // Test Type: ChannelError
        for (uint8_t i = 1; i < (uint8_t)(ChannelDefinition::ChannelErrorType::END_OF_LIST); ++i) {
            EXPECT_FALSE(ChannelDefinition::ChannelErrorString(
                             (ChannelDefinition::ChannelErrorType)(i)) == "UNKNOWN");
        }

        // Test Type: ChannelType
        for (uint8_t i = 1; i < (uint8_t)(ChannelDefinition::ChannelType::END_OF_LIST); ++i) {
            EXPECT_FALSE(ChannelDefinition::ChannelTypeString(
                             (ChannelDefinition::ChannelType)(i)) == "UNKNOWN");
        }

        // Test Type: Direction
        for (uint8_t i = 1; i < (uint8_t)(ChannelDefinition::Direction::END_OF_LIST); ++i) {
            EXPECT_FALSE(ChannelDefinition::ChannelDirectionString(
                             (ChannelDefinition::Direction)(i)) == "UNKNOWN");
        }
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}