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

#include <map>
namespace ros_hats {
class IServoHatDriver
{
   public:
    struct Channel {
        Channel(uint8_t pin_number, std::string name, uint16_t value)
            : pin_number(pin_number), name(name), value(value) {
        }

        uint8_t pin_number;
        std::string name;
        uint16_t value;
    };
    static constexpr int MIN_SERVO_VALUE = 500;
    static constexpr int MEDIUM_SERVO_VALUE = 1000;
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

    virtual bool setServoValue(int pin_number, int v) = 0;
    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    virtual bool finish() = 0;
    virtual std::string pretty(std::string mode = "") = 0;
    virtual std::map<uint8_t, Channel> get_channels() = 0;
};
}  // namespace ros_hats