/**
 * @file IServoHatDriver.h
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

namespace ros_hats {
class IServoHatDriver
{
   public:
    static constexpr int MIN_SERVO_VALUE = 500;
    static constexpr int MAX_SERVO_VALUE = 1500;
    struct ServoHatDriverContainer {
        ros::Time timestamp;
    };

    IServoHatDriver() {
    }
    virtual ~IServoHatDriver(){};
    /**
     * @brief Initialize Servo Hat Driver
     *
     * @param logger
     * @return true
     * @return false
     */
    virtual bool init(eros::Logger* logger, int address = 0x40) = 0;

    virtual bool update(double dt) = 0;

    virtual void setServoValue(int pin_number, int v) = 0;
    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    virtual bool finish() = 0;
    virtual std::string pretty() = 0;
};
}  // namespace ros_hats