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
    Channel(ChannelConfig _config) : channel_config(_config) {
    }

    ~Channel();

    bool base_init();
    virtual bool init() = 0;

    std::string base_pretty(std::string pre);
    virtual std::string pretty(std::string pre) = 0;

    ChannelDefinition::ChannelType get_channel_type() {
        return channel_config.channel_type;
    }
    ChannelDefinition::Direction get_direction() {
        return channel_config.direction;
    }
    std::string get_channel_name() {
        return channel_config.channel_name;
    }
    uint16_t get_pin_number() {
        return channel_config.pin_number;
    }

   protected:
    ChannelConfig channel_config;
};
#endif  // ROSHATS_CHANNEL_H