#include <ros_hats/Port/Port.h>
Port::Port() {
}
Port::~Port() {
}
std::string Port::base_pretty() {
    std::string str;
    str = "\tPort: " + port_config.port_name +
          " Type: " + ChannelDefinition::ChannelTypeString(port_config.port_type) + "\n";
    if (channels.size() == 0) {
        str += "  NO Channels Defined. \n";
    }
    else {
        for (const auto &p : channels) { str += "\t\tChannel" + p.second->pretty(); }
    }
    return str;
}
bool Port::base_init() {
    return true;
}