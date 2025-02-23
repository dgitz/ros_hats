/**
 * @file ServoHatDriver.h
 * @author David Gitz
 * @brief
 * @date 2025-02-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/Logger.h>
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/PrettyUtility.h>
#include <wiringPiI2C.h>

#include "BaseServoHatDriver.h"

//! ros_hats Namespace
namespace ros_hats {
/**
 * @brief ServoHatDriver Class
 * @details Connects to an instance of a Servo Hat
 *
 */
class ServoHatDriver : public BaseServoHatDriver
{
   public:
    enum class Adafruit16ChServoHatConstant {
        MODE1 = 0x00,
        MODE2 = 0x01,
        SUBADR1 = 0x02,
        SUBADR2 = 0x03,
        SUBADR3 = 0x04,
        PRESCALE = 0xFE,
        LED0_ON_L = 0x06,
        LED0_ON_H = 0x07,
        LED0_OFF_L = 0x08,
        LED0_OFF_H = 0x09,
        ALL_LED_ON_L = 0xFA,
        ALL_LED_ON_H = 0xFB,
        ALL_LED_OFF_L = 0xFC,
        ALL_LED_OFF_H = 0xFD,
        RESTART = 0x80,
        SLEEP = 0x10,
        ALLCALL = 0x01,
        INVRT = 0x10,
        OUTDRV = 0x04,
    };
    ServoHatDriver();
    virtual ~ServoHatDriver();
    bool init(eros::Logger* logger, int address = 0x40) override;
    bool setServoValue(int pin_number, int v) override;
    bool finish() override;

    std::string pretty(std::string mode) override;

   private:
    void setPWMFreq(int freq);

    void setPWM(int pin_number, int on, int off);

    void resetAllPWM(int on, int off);

    void resetAllServo();
    eros::Logger* logger;
    int servoHatFd;
};
}  // namespace ros_hats