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

#include "IServoHatDriver.h"

//! ros_hats Namespace
namespace ros_hats {
/**
 * @brief MockServoHatDriver Class
 * @details
 *
 */
class MockServoHatDriver : public IServoHatDriver
{
   public:
    MockServoHatDriver();
    virtual ~MockServoHatDriver();

    bool init(eros::Logger* logger, int address = 0x40) override;
    bool update(double dt) override;
    void setServoValue(int pin_number, int v) override;
    bool finish() override;

    std::string pretty() override;

   private:
    eros::Logger* logger;
};
}  // namespace ros_hats