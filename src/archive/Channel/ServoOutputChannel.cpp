#include <ros_hats/Channel/ServoOutputChannel.h>
ServoOutputChannel::~ServoOutputChannel() {
}
bool ServoOutputChannel::init() {
    bool v = base_init();
    return v;
}
std::string ServoOutputChannel::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += " V: " + std::to_string(value) + " Update Count: " + std::to_string(update_count);
    return str;
}