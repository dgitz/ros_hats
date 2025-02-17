#include "GPSHatDriver.h"
using namespace ros_hats;
GPSHatDriver driver;
int main() {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_GPSHatDriver");
    logger->log_debug("Starting GPS Driver");

    driver.init(logger);
    double delta_time_sec = 0.25;
    while (true) {
        driver.update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
        logger->log_info(driver.pretty());
    }

    logger->log_debug("GPS Driver Finished.");
    delete logger;
    return 0;
}