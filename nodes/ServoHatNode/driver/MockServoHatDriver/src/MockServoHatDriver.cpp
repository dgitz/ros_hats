#include "MockServoHatDriver.h"
namespace ros_hats {
MockServoHatDriver::MockServoHatDriver() {
}
MockServoHatDriver::~MockServoHatDriver() {
    finish();
}
bool MockServoHatDriver::finish() {
    logger->log_warn("Finish");
    return true;
}
bool MockServoHatDriver::init(eros::Logger* _logger, int /* address*/) {
    logger = _logger;
    logger->log_warn("This is a Mock of the Servo Hat!");
    channel_map.insert(std::pair<uint8_t, Channel>(
        0, Channel(0, "Mock Pin", IServoHatDriver::MEDIUM_SERVO_VALUE)));
    return true;
}
std::string MockServoHatDriver::pretty(std::string mode) {
    std::string str = "Mock ServoHatDriver" + BaseServoHatDriver::pretty(mode);
    return str;
}
bool MockServoHatDriver::setServoValue(int channel, int v) {
    if (BaseServoHatDriver::setServoValue(channel, v) == false) {
        logger->log_warn("Not able to update Channel: " + std::to_string(channel));
        return false;
    }
    return true;
}
}  // namespace ros_hats