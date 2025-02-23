#ifdef ARCHITECTURE_ARMV7L
#include "ServoHatDriver.h"
#else
#include "MockServoHatDriver.h"
#endif
using namespace ros_hats;
void printHelp() {
    printf("Tester for Servo Hat Driver\n");
    printf("-h This Menu.\n");
    printf("-r Reset all Channels.\n");
    printf("-c Channel Number.\n");
    printf("-m Mode: ramp,direct.\n");
    printf("-v Value to Set.\n");
}
int main(int argc, char* argv[]) {
    eros::Logger* logger = new eros::Logger("DEBUG", "exec_ServoHatDriver");
    bool reset = false;
    int channel = 0;
    int value = 0;
    std::string mode = "direct";  // default
    for (;;) {
        switch (getopt(argc,
                       argv,
                       "rc:m:v:h"))  // note the colon (:) to indicate that 'b' has a parameter and
                                     // is not a switch
        {
            case 'r': reset = true; break;

            case 'c': channel = atoi(optarg); continue;
            case 'm': mode = optarg; break;
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
    logger->log_notice(driver->pretty());
    logger->log_notice(driver->pretty("simple"));
    double delta_time_sec = 0.01;
    if (reset == true) {
        for (uint8_t ch = 0; ch < 16; ++ch) { driver->setServoValue(ch, 1000); }
        logger->log_notice("Reset Complete");
        delete logger;
        delete driver;
        return 0;
    }
    else if (mode == "direct") {
    }
    else if (mode == "ramp") {
        value = IServoHatDriver::MIN_SERVO_VALUE;
    }
    else {
        logger->log_error("Mode: " + mode + " Not Supported!");
        delete logger;
        delete driver;
        return 1;
    }

    bool direction = true;

    while (true) {
        driver->update(delta_time_sec);
        usleep(delta_time_sec * 1000000);
        if (mode == "ramp") {
            if (value >= IServoHatDriver::MAX_SERVO_VALUE) {
                direction = false;
            }
            else if (value <= IServoHatDriver::MIN_SERVO_VALUE) {
                direction = true;
            }
            if (direction == true) {
                value += 5;
            }
            else {
                value -= 5;
            }
        }
        else if (mode == "direct") {  // Default, nothing to do here
        }

        driver->setServoValue(channel, value);
        logger->log_debug(driver->pretty("simple"));
    }

    logger->log_debug("Servo Hat Driver Finished.");
    delete logger;
    delete driver;
    return 0;
}