/*! \file PWMOutputChannel.h
 */
#ifndef ROSHATS_PWMOUTPUT_CHANNEL_H
#define ROSHATS_PWMOUTPUT_CHANNEL_H
#include <ros_hats/Channel/Channel.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class PWMOutputChannel
    \brief PWMOutputChannel class
    Basic container for a PWMOutputChannel
*/
class PWMOutputChannel : public Channel
{
   public:
    PWMOutputChannel() {
    }
    PWMOutputChannel(ChannelConfig _config) : update_count(0) {
        channel_config = _config;
        auto data_config =
            std::static_pointer_cast<PWMChannelDataConfig>(channel_config.data_config);
        value = data_config->default_value;
        default_value = value;
        upper_range = data_config->max_value;
        lower_range = data_config->min_value;
        channel_config.channel_type = ChannelDefinition::ChannelType::PWM;
        channel_config.direction = ChannelDefinition::Direction::OUTPUT;
    }
    ~PWMOutputChannel();

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
    int64_t default_value;
    int64_t lower_range;
    int64_t upper_range;
    uint64_t update_count;
};
#endif  // ROSHATS_PWMOUTPUT_CHANNEL_H