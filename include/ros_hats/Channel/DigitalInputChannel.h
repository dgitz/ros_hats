/*! \file DigitalInputChannel.h
 */
#ifndef ROSHATS_DIGITALINPUT_CHANNEL_H
#define ROSHATS_DIGITALINPUT_CHANNEL_H
#include <ros_hats/Channel/Channel.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class DigitalInputChannel
    \brief DigitalInputChannel class
    Basic container for a DigitalInputChannel
*/
class DigitalInputChannel : public Channel
{
   public:
    DigitalInputChannel() {
    }
    DigitalInputChannel(ChannelConfig _config) : update_count(0) {
        channel_config = _config;
        auto data_config = std::static_pointer_cast<DigitalChannelDataConfig>(_config.data_config);
        value = data_config->default_value;
        default_value = value;
        upper_range = data_config->max_value;
        lower_range = data_config->min_value;
        channel_config.channel_type = ChannelDefinition::ChannelType::DIGITAL;
        channel_config.direction = ChannelDefinition::Direction::CH_INPUT;
    }
    ~DigitalInputChannel();

    bool init();

    std::string pretty(std::string pre);

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
        else if (v < lower_range) {
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
#endif  // ROSHATS_DIGITALINPUT_CHANNEL_H