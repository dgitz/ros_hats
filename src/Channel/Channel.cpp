#include <ros_hats/Channel/Channel.h>

Channel::~Channel() {
}
std::string Channel::base_pretty() {
    std::string str;
    str = "  Name: " + name + " Pin Name: " + pin_name + "\n";
    str += "  Type: " + ChannelDefinition::ChannelTypeString(channel_type) + "\n";
    str += "  Direction: " + ChannelDefinition::ChannelDirectionString(direction) + "\n";
    return str;
}
bool Channel::base_init() {
    return true;
}