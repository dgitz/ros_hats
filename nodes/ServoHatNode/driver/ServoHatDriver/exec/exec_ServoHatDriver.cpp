#include "ServoHatDriver.h"
using namespace ros_hats;
ServoHatDriver driver;
int main() {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_ServoHatDriver");
    logger->log_debug("Starting Servo Hat Driver");

    bool status = driver.init(logger);
    if(status == false) {
        logger->log_error("Unable to initialize Driver!  Exiting.");
        return 1;
    }
    double delta_time_sec = 0.1;
    while (true) {
        driver.update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
        logger->log_debug(driver.pretty());
    }

    logger->log_debug("GPS Driver Finished.");
    delete logger;
    return 0;
}