#include <ros_hats/Channel/Channel.h>

Channel::~Channel() {
}
std::string Channel::base_pretty(std::string pre) {
    std::string str;
    str = pre + "Channel: " + channel_config.channel_name +
          " Pin Number: " + std::to_string(channel_config.pin_number) +
          " Type: " + ChannelDefinition::ChannelTypeString(channel_config.channel_type) +
          " Direction: " + ChannelDefinition::ChannelDirectionString(channel_config.direction);
    return str;
}
bool Channel::base_init() {
    return true;
}