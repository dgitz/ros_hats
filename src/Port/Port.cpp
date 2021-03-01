#include <ros_hats/Port/Port.h>
Port::Port() {
}
Port::~Port() {
}
std::string Port::base_pretty() {
    std::string str;
    str = "  Port: " + name + "\n";
    str += "  Type: " + ChannelDefinition::ChannelTypeString(port_type) + "\n";
    if (channels.size() == 0) {
        str += "  NO Channels Defined. \n";
    }
    else {
        for (const auto &p : channels) { str += "Channel" + p.second->pretty(); }
    }
    return str;
}
bool Port::base_init() {
    return true;
}