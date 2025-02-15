#ifndef GPIODRIVER_h
#define GPIODRIVER_H
#include <ros_hats/ROSHATS_Definitions.h>
#include <wiringPi.h>

#include <fstream>
#include <string>

class GPIODriver
{
   public:
    const uint64_t DELAY_GPIOEXPORT_MS = 60000; /*!< How long to allow GPIO sysfs to update. */
    GPIODriver();
    ~GPIODriver();

    bool init(std::string _gpio_pin_name,
              ChannelDefinition::ChannelType _channel_type,
              ChannelDefinition::Direction _channel_direction);
    static void inputCB();

    bool setdir_gpio(std::string _gpio_pin_name, std::string dir);
    bool setvalue_gpio(std::string _gpio_pin_name, std::string value);
    bool cleanup();
    int number;

   private:
    bool export_gpio(std::string _gpio_pin_name);
    bool unexport_gpio(std::string _gpio_pin_name);
    uint16_t pin_number;
    std::string gpio_pin_name;
    ChannelDefinition::ChannelType channel_type;
    ChannelDefinition::Direction channel_direction;
};
#endif  // GPIODRIVER_H