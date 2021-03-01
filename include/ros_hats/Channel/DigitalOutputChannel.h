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
    DigitalOutputChannel(std::string _name,
                         std::string _pin_name,
                         int64_t _default_value,
                         int64_t _lower_range,
                         int64_t _upper_range)
        : Channel(_name,
                  _pin_name,
                  ChannelDefinition::ChannelType::DIGITAL,
                  ChannelDefinition::Direction::OUTPUT),
          value(_default_value),
          lower_range(_lower_range),
          upper_range(_upper_range),
          update_count(0),
          default_value(_default_value) {
    }
    ~DigitalOutputChannel();

    bool init();

    std::string pretty();

    int64_t get_value() {
        return value;
    }
    int64_t get_default_value() {
        return default_value;
    }
    ChannelDefinition::ChannelErrorType update_value(int64_t v) {
        if (v > upper_range) {
            value = upper_range;
            update_count++;
            return ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND;
        }
        else if (v = lower_range) {
            value = lower_range;
            update_count++;
            return ChannelDefinition::ChannelErrorType::VALUE_EXCEED_LOWER_BOUND;
        }
        else {
            value = v;
            update_count++;
            return ChannelDefinition::ChannelErrorType::NOERROR;
        }
    }

   private:
    int64_t value;
    int64_t lower_range;
    int64_t upper_range;
    uint64_t update_count;
    int64_t default_value;
};
#endif  // ROSHATS_DIGITALOUTPUT_CHANNEL_H