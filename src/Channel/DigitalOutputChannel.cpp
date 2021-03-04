#include <ros_hats/Channel/DigitalOutputChannel.h>
DigitalOutputChannel::~DigitalOutputChannel() {
}
bool DigitalOutputChannel::init() {
    bool v = base_init();
    return v;
}
std::string DigitalOutputChannel::pretty() {
    std::string str = base_pretty();
    str +=
        "  V: " + std::to_string(value) + " Update Count: " + std::to_string(update_count) + "\n";
    return str;
}