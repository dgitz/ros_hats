#ifndef Adafruit16ChServoHat_h
#define Adafruit16ChServoHat_h

#include <math.h>
#include <time.h>
#include <wiringPiI2C.h>

#include <map>
#include <string>
class Adafruit16ChServoHat
{
   public:
    enum Adafruit16ChServoHatConstant {
        __MODE1 = 0x00,
        __MODE2 = 0x01,
        __SUBADR1 = 0x02,
        __SUBADR2 = 0x03,
        __SUBADR3 = 0x04,
        __PRESCALE = 0xFE,
        __LED0_ON_L = 0x06,
        __LED0_ON_H = 0x07,
        __LED0_OFF_L = 0x08,
        __LED0_OFF_H = 0x09,
        __ALL_LED_ON_L = 0xFA,
        __ALL_LED_ON_H = 0xFB,
        __ALL_LED_OFF_L = 0xFC,
        __ALL_LED_OFF_H = 0xFD,

        __RESTART = 0x80,
        __SLEEP = 0x10,

        __ALLCALL = 0x01,
        __INVRT = 0x10,
        __OUTDRV = 0x04,
    };

    Adafruit16ChServoHat();
    ~Adafruit16ChServoHat();
    int init(int _address = 0x40);
    void setPWMFreq(int freq);

    void setPWM(int pin_number, int on, int off);
    void setServoValue(int pin_number, int v);
    void resetAllPWM(int on, int off);

    void resetAllServo();
    int get_address() {
        return address;
    }

   private:
    int address;
    int ServoHatfd;
};

#endif  // Adafruit16ChServoHat_h
