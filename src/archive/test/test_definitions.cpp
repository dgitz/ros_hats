/*! \file test_definitions.cpp
 */
#include <gtest/gtest.h>
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
TEST(BasicTest, TestDefintions) {
    // Test Class: RaspberryPiDefinition
    {
        // {
        // Test Type: RaspberryPiModel
        for (uint8_t i = 1; i < (uint8_t)(RaspberryPiDefinition::RaspberryPiModel::END_OF_LIST);
             ++i) {
            EXPECT_FALSE(RaspberryPiDefinition::RaspberryPiModelString(
                             (RaspberryPiDefinition::RaspberryPiModel)(i)) == "UNKNOWN");
        }
        for (uint8_t i = 1; i < (uint8_t)(RaspberryPiDefinition::PinType::END_OF_LIST); ++i) {
            EXPECT_FALSE(RaspberryPiDefinition::PinTypeString(
                             (RaspberryPiDefinition::PinType)(i)) == "UNKNOWN");
        }

        std::vector<RaspberryPiDefinition::RaspberryPi> devices =
            RaspberryPiDefinition::build_RaspberryPi();
        EXPECT_TRUE(devices.size() > 0);
        for (auto device : devices) {
            printf("%s\n", RaspberryPiDefinition::pretty(device).c_str());
            EXPECT_TRUE(device.pin_defines.size() > 0);
            EXPECT_TRUE(device.pin_defines.size() == device.pin_map.size());
            for (auto pin : device.pin_map) {
                auto find = device.pin_defines.find(pin.left);
                EXPECT_TRUE(find != device.pin_defines.end());
            }
        }
    }
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