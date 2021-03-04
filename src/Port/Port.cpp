#include <ros_hats/Port/Port.h>
Port::Port() {
}
Port::~Port() {
}
std::string Port::base_pretty(std::string pre) {
    std::string str;
    str = pre + "Port: " + port_config.port_name +
          " Type: " + ChannelDefinition::ChannelTypeString(port_config.port_type) + "\n";
    return str;
}
bool Port::base_init() {
    return true;
}