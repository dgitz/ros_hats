#include <ros_hats/Channel/DigitalInputChannel.h>
DigitalInputChannel::~DigitalInputChannel() {
}
bool DigitalInputChannel::init() {
    bool v = base_init();
    return v;
}
std::string DigitalInputChannel::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += " V: " + std::to_string(value) + " Update Count: " + std::to_string(update_count);
    return str;
}