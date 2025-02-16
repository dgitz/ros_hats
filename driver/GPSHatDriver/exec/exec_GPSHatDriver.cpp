#include "../include/GPSHatDriver.h"
using namespace ros_hats;
GPSHatDriver driver;
int main() {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_GPSHatDriver");
    logger->log_debug("Starting GPS Driver");

    driver.init(logger);
    while (true) {
        driver.update(1.0);
        sleep(1);
    }

    logger->log_debug("GPS Driver Finished.");
    delete logger;
    return 0;
}