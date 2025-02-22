#include "ServoHatDriver.h"
using namespace ros_hats;
ServoHatDriver driver;
int main() {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_ServoHatDriver");
    logger->log_debug("Starting Servo Hat Driver");

    bool status = driver.init(logger);
    if (status == false) {
        logger->log_error("Unable to initialize Driver!  Exiting.");
        return 1;
    }
    double delta_time_sec = 0.1;
    int MIN_VALUE = 50;
    int MAX_VALUE = 150;
    int value = MIN_VALUE;
    bool direction = true;
    while (true) {
        driver.update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
        logger->log_debug(driver.pretty());
        if (value >= MAX_VALUE) {
            direction = false;
        }
        if (value <= MIN_VALUE) {
            direction = true;
        }
        if (direction) {
            value += 10;
        }
        else {
            value -= 10;
        }
        logger->log_debug("V: " + std::to_string(value));
        driver.setServoValue(0, value);
    }

    logger->log_debug("GPS Driver Finished.");
    delete logger;
    return 0;
}