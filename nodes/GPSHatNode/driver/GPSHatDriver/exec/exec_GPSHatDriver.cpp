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
        switch (driver.get_level(driver.get_gps_data().status_type)) {
            case eros::Level::Type::DEBUG: logger->log_debug(driver.pretty()); break;
            case eros::Level::Type::INFO: logger->log_info(driver.pretty()); break;
            case eros::Level::Type::NOTICE: logger->log_notice(driver.pretty()); break;
            case eros::Level::Type::WARN: logger->log_warn(driver.pretty()); break;
            case eros::Level::Type::ERROR: logger->log_error(driver.pretty()); break;
            default: logger->log_error(driver.pretty()); break;
        }
    }

    logger->log_debug("GPS Driver Finished.");
    delete logger;
    return 0;
}