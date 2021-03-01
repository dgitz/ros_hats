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
            ChannelDefinition::ChannelType _channel_type,
            ChannelDefinition::Direction _direction)
        : name(_name), pin_name(_pin_name), channel_type(_channel_type), direction(_direction) {
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

   protected:
    std::string name;
    std::string pin_name;
    ChannelDefinition::ChannelType channel_type;
    ChannelDefinition::Direction direction;
};
#endif  // ROSHATS_CHANNEL_H