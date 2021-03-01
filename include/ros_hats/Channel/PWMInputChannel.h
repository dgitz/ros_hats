/*! \file PWMInputChannel.h
 */
#ifndef ROSHATS_PWMINPUT_CHANNEL_H
#define ROSHATS_PWMINPUT_CHANNEL_H
#include <ros_hats/Channel/Channel.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class PWMInputChannel
    \brief PWMInputChannel class
    Basic container for a PWMInputChannel
*/
class PWMInputChannel : public Channel
{
   public:
    PWMInputChannel() {
    }
    PWMInputChannel(std::string _name,
                    std::string _pin_name,
                    int64_t default_value,
                    int64_t _lower_range,
                    int64_t _upper_range)
        : Channel(_name,
                  _pin_name,
                  ChannelDefinition::ChannelType::PWM,
                  ChannelDefinition::Direction::INPUT),
          value(default_value),
          lower_range(_lower_range),
          upper_range(_upper_range),
          update_count(0) {
    }
    ~PWMInputChannel();

    bool init();

    std::string pretty();

    int64_t get_value() {
        return value;
    }
    ChannelDefinition::ChannelErrorType update_value(int64_t v) {
        if (v >= upper_range) {
            value = upper_range;
            update_count++;
            return ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND;
        }
        else if (v <= lower_range) {
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
};
#endif  // ROSHATS_PWMINPUT_CHANNEL_H