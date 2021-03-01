#include <ros_hats/Channel/PWMInputChannel.h>
PWMInputChannel::~PWMInputChannel() {
}
bool PWMInputChannel::init() {
    bool v = base_init();
    return v;
}
std::string PWMInputChannel::pretty() {
    std::string str = base_pretty();
    return str;
}