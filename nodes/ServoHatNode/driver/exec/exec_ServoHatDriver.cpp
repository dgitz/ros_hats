#ifdef ARCHITECTURE_ARMV7L
#include "ServoHatDriver.h"
#else
#include "MockServoHatDriver.h"
#endif
using namespace ros_hats;
void printHelp() {
    printf("Tester for Servo Hat Driver\n");
    printf("-h This Menu.\n");
    printf("-c Channel Number.\n");
    printf("-m Mode: Ramp,Direct.\n");
    printf("-v Value to Set.\n");
}
int main(int argc, char* argv[]) {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_ServoHatDriver");
    int channel = 0;
    int value = 0;
    bool ramp = false;
    for (;;) {
        switch (getopt(argc,
                       argv,
                       "c:m:v:h"))  // note the colon (:) to indicate that 'b' has a parameter and
                                    // is not a switch
        {
            case 'c': channel = atoi(optarg); continue;
            case 'm': ramp = true; break;
            case 'v': value = atoi(optarg); break;
            case '?': printHelp(); return 0;
            case 'h': printHelp(); return 0;
            default: printHelp(); return 0;
        }

        break;
    }
    IServoHatDriver* driver;
#ifdef ARCHITECTURE_ARMV7L
    driver = new ServoHatDriver();
#else
    driver = new MockServoHatDriver();
#endif
    driver->init(logger);
    double delta_time_sec = 0.1;
    if (ramp == true) {
        value = IServoHatDriver::MIN_SERVO_VALUE;
    }
    bool direction = true;
    while (true) {
        driver->update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
        logger->log_debug("Channel: " + std::to_string(channel) +
                          " Value: " + std::to_string(value));
        if (ramp == true) {
            if (value >= IServoHatDriver::MAX_SERVO_VALUE) {
                direction = false;
            }
            else if (value <= IServoHatDriver::MIN_SERVO_VALUE) {
                direction = true;
            }
            if (direction == true) {
                value += 50;
            }
            else {
                value -= 50;
            }
        }

        driver->setServoValue(channel, value);
    }

    logger->log_debug("Servo Hat Driver Finished.");
    delete logger;
    delete driver;
    return 0;
}