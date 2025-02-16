/*! \file test_raspberryPiDefinition.cpp
 */
#include <gtest/gtest.h>
#include <ros_hats/RaspberryPiDefinition.h>
#include <stdio.h>
using namespace ros_hats;
TEST(BasicTest, TestDefinitions) {
    for (uint8_t i = 0; i < (uint8_t)RaspberryPiDefinition::RaspberryPiModel::END_OF_LIST; ++i) {
        if ((i == (uint8_t)RaspberryPiDefinition::RaspberryPiModel::UNKNOWN) ||
            (i == (uint8_t)RaspberryPiDefinition::RaspberryPiModel::END_OF_LIST)) {
            EXPECT_EQ(RaspberryPiDefinition::RaspberryPiModelString(
                          (RaspberryPiDefinition::RaspberryPiModel)i),
                      "UNKNOWN");
        }
        else {
            EXPECT_NE(RaspberryPiDefinition::RaspberryPiModelString(
                          (RaspberryPiDefinition::RaspberryPiModel)i),
                      "UNKNOWN");
        }
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}