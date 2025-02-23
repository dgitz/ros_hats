#include "ServoHatDriver.h"
namespace ros_hats {
ServoHatDriver::ServoHatDriver() {
}
ServoHatDriver::~ServoHatDriver() {
    finish();
}
bool ServoHatDriver::finish() {
    logger->log_warn("Finish");
    resetAllPWM(0, 0);
    return true;
}
bool ServoHatDriver::init(eros::Logger* _logger, int address) {
    logger = _logger;
    servoHatFd = wiringPiI2CSetup(address);
    resetAllPWM(0, 0);

    wiringPiI2CWriteReg8(servoHatFd,
                         (int)Adafruit16ChServoHatConstant::MODE2,
                         (int)Adafruit16ChServoHatConstant::OUTDRV);
    wiringPiI2CWriteReg8(servoHatFd,
                         (int)Adafruit16ChServoHatConstant::MODE1,
                         (int)Adafruit16ChServoHatConstant::ALLCALL);

    int mode1 = wiringPiI2CReadReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::MODE1);
    if (mode1 < 0) {
        return false;
    }
    mode1 = mode1 & ~(int)Adafruit16ChServoHatConstant::SLEEP;
    if (mode1 < 0) {
        return false;
    }
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::MODE1, mode1);

    setPWMFreq(60);
    for (uint8_t ch = 0; ch < 16; ++ch) {
        channel_map.insert(std::pair<uint8_t, Channel>(
            ch, Channel(ch, "CH" + std::to_string(ch), IServoHatDriver::MEDIUM_SERVO_VALUE)));
    }

    return true;
}
std::string ServoHatDriver::pretty(std::string mode) {
    std::string str = "ServoHatDriver" + BaseServoHatDriver::pretty(mode);
    return str;
}
bool ServoHatDriver::setServoValue(int channel, int v) {
    if (BaseServoHatDriver::setServoValue(channel, v) == false) {
        logger->log_warn("Not able to set Channel: " + std::to_string(channel));
        return false;
    }

    int on = 0;
    int off = v / 3.90;
    setPWM(channel, on, off);
    return true;
}
void ServoHatDriver::setPWMFreq(int freq) {
    float prescaleval = 25000000;
    prescaleval /= 4096.0;
    prescaleval /= (float)freq;
    prescaleval -= 1.0;
    int prescale = floor(prescaleval + 0.5);

    int oldmode = wiringPiI2CReadReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::MODE1);
    int newmode = (oldmode & 0x7F) | 0x10;
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::MODE1, newmode);
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::PRESCALE, floor(prescale));
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::MODE1, oldmode);

    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::MODE1, oldmode | 0x80);
}

void ServoHatDriver::resetAllServo() {
}
void ServoHatDriver::setPWM(int channel, int on, int off) {
    wiringPiI2CWriteReg8(
        servoHatFd, (int)Adafruit16ChServoHatConstant::LED0_ON_L + 4 * channel, on & 0xFF);
    wiringPiI2CWriteReg8(
        servoHatFd, (int)Adafruit16ChServoHatConstant::LED0_ON_H + 4 * channel, on >> 8);
    wiringPiI2CWriteReg8(
        servoHatFd, (int)Adafruit16ChServoHatConstant::LED0_OFF_L + 4 * channel, off & 0xFF);
    wiringPiI2CWriteReg8(
        servoHatFd, (int)Adafruit16ChServoHatConstant::LED0_OFF_H + 4 * channel, off >> 8);
}

void ServoHatDriver::resetAllPWM(int on, int off) {
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::ALL_LED_ON_L, on & 0xFF);
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::ALL_LED_ON_H, on >> 8);
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::ALL_LED_OFF_L, off & 0xFF);
    wiringPiI2CWriteReg8(servoHatFd, (int)Adafruit16ChServoHatConstant::ALL_LED_OFF_H, off >> 8);
}
}  // namespace ros_hats