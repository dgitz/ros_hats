/**
 * @file MockServoHatDriver.h
 * @author David Gitz
 * @brief
 * @date 2025-02-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "BaseServoHatDriver.h"

//! ros_hats Namespace
namespace ros_hats {
/**
 * @brief MockServoHatDriver Class
 * @details
 *
 */
class MockServoHatDriver : public BaseServoHatDriver
{
   public:
    MockServoHatDriver();
    virtual ~MockServoHatDriver();

    bool init(eros::Logger* logger, int address = 0x40) override;
    bool setServoValue(int pin_number, int v) override;
    bool finish() override;

    std::string pretty(std::string mode) override;
};
}  // namespace ros_hats