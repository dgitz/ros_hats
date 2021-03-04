#include <ros_hats/Channel/Channel.h>

Channel::~Channel() {
}
std::string Channel::base_pretty() {
    std::string str;
    str = "  Name: " + channel_config.channel_name +
          " Pin Number: " + std::to_string(channel_config.pin_number) + "\n";
    str += "  Type: " + ChannelDefinition::ChannelTypeString(channel_config.channel_type) + "\n";
    str += "  Direction: " + ChannelDefinition::ChannelDirectionString(channel_config.direction) +
           "\n";
    return str;
}
bool Channel::base_init() {
    return true;
}