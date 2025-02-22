#ifdef ARCHITECTURE_ARMV7L
#include "ServoHatDriver.h"
#else
#include "MockServoHatDriver.h"
#endif
using namespace ros_hats;

int main() {
    IServoHatDriver* driver;
#ifdef ARCHITECTURE_ARMV7L
    driver = new ServoHatDriver();
#else
    driver = new MockServoHatDriver();
#endif

    eros::Logger* logger = new eros::Logger("DEBUG", "exec_ServoHatDriver");
    logger->log_debug("Starting Servo Hat Driver");

    bool status = driver->init(logger);
    if (status == false) {
        logger->log_error("Unable to initialize Driver!  Exiting.");
        return 1;
    }
    double delta_time_sec = 0.1;
    int value = IServoHatDriver::MIN_SERVO_VALUE - 100;
    bool direction = true;
    while (true) {
        driver->update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
        logger->log_warn(driver->pretty());
        if (value >= IServoHatDriver::MAX_SERVO_VALUE + 100) {
            direction = false;
        }
        if (value <= IServoHatDriver::MIN_SERVO_VALUE - 100) {
            direction = true;
        }
        if (direction) {
            value += 50;
        }
        else {
            value -= 50;
        }

        logger->log_debug("V: " + std::to_string(value));
        driver->setServoValue(0, value);
    }

    logger->log_debug("Servo Hat Driver Finished.");
    delete logger;
    delete driver;
    return 0;
}