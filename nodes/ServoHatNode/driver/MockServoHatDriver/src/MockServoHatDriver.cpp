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
bool MockServoHatDriver::init(eros::Logger* _logger, int address) {
    logger = _logger;
    logger->log_warn("This is a Mock of the Servo Hat!");

    return true;
}
bool MockServoHatDriver::update(double dt) {
    return true;
}
std::string MockServoHatDriver::pretty() {
    std::string str;
    str = "Not Implemented Yet";
    return str;
}
void MockServoHatDriver::setServoValue(int channel, int v) {
    logger->log_warn("Not Implemented Yet");
}
}  // namespace ros_hats