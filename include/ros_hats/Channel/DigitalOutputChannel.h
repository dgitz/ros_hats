/*! \file DigitalOutputChannel.h
 */
#ifndef ROSHATS_DIGITALOUTPUT_CHANNEL_H
#define ROSHATS_DIGITALOUTPUT_CHANNEL_H
#include <ros_hats/Channel/Channel.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class DigitalOutputChannel
    \brief DigitalOutputChannel class
    Basic container for a DigitalOutputChannel
*/
class DigitalOutputChannel : public Channel
{
   public:
    DigitalOutputChannel() {
    }
    DigitalOutputChannel(std::string _name, std::string _pin_name)
        : Channel(_name,
                  _pin_name,
                  ChannelDefinition::ChannelType::DIGITAL,
                  ChannelDefinition::Direction::OUTPUT) {
    }
    ~DigitalOutputChannel();

    bool init();

    std::string pretty();

   private:
};
#endif  // ROSHATS_DIGITALOUTPUT_CHANNEL_H