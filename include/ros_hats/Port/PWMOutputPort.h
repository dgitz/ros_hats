/*! \file PWMOutputPort.h
 */
#ifndef ROSHATS_PWMOUTPUTPORT_H
#define ROSHATS_PWMOUTPUTPORT_H
#include <ros_hats/Channel/PWMOutputChannel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class PWMOutputPort
    \brief PWMOutputPort class
    Basic container for a PWMOutputPort
*/
class PWMOutputPort : public Port
{
   public:
    PWMOutputPort();
    PWMOutputPort(std::string _name, std::vector<std::string> _pin_names)
        : Port(_name,
               _pin_names,
               ChannelDefinition::ChannelType::PWM,
               ChannelDefinition::Direction::OUTPUT) {
        for (uint16_t i = 0; i < port_size; ++i) {
            channels.emplace(std::make_pair(
                _pin_names.at(i),
                new PWMOutputChannel(
                    _name + "_" + std::to_string(i), _pin_names.at(i), 1500, 1000, 2000)));
        }
    }
    ~PWMOutputPort();
    bool init();
    ChannelDefinition::ChannelErrorType update(std::string pin_name, int64_t value);
    int64_t get_value(std::string pin_name);
    std::string pretty();

   private:
};
#endif  // ROSHATS_PWMOUTPUTPORT_H