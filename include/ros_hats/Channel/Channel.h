/*! \file Channel.h
 */
#ifndef ROSHATS_CHANNEL_H
#define ROSHATS_CHANNEL_H
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class Channel
    \brief Channel class
    Basic container for a Channel
*/
class Channel
{
   public:
    Channel() {
    }
    Channel(std::string _name,
            std::string _pin_name,
            uint16_t _pin_number,
            ChannelDefinition::ChannelType _channel_type,
            ChannelDefinition::Direction _direction)
        : name(_name),
          pin_name(_pin_name),
          pin_number(_pin_number),
          channel_type(_channel_type),
          direction(_direction) {
    }

    ~Channel();

    bool base_init();
    virtual bool init() = 0;

    std::string base_pretty();
    virtual std::string pretty() = 0;

    ChannelDefinition::ChannelType get_channel_type() {
        return channel_type;
    }
    ChannelDefinition::Direction get_direction() {
        return direction;
    }
    std::string get_pin_name() {
        return pin_name;
    }
    uint16_t get_pin_number() {
        return pin_number;
    }

   protected:
    std::string name;
    std::string pin_name;
    uint16_t pin_number;
    ChannelDefinition::ChannelType channel_type;
    ChannelDefinition::Direction direction;
};
#endif  // ROSHATS_CHANNEL_H