#include <ros_hats/Port/PWMOutputPort.h>
PWMOutputPort::PWMOutputPort() {
}
PWMOutputPort::~PWMOutputPort() {
}
std::string PWMOutputPort::pretty() {
    std::string str = base_pretty();
    return str;
}
bool PWMOutputPort::init() {
    bool v = base_init();
    return v;
}
ChannelDefinition::ChannelErrorType PWMOutputPort::update(std::string pin_name, int64_t value) {
    auto found = channels.find(pin_name);
    if (found != channels.end()) {
        PWMOutputChannel *channel = dynamic_cast<PWMOutputChannel *>(found->second.get());
        if (channel == nullptr) {
            return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
        }
        return channel->update_value(value);
    }
    else {
        return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    }
}
int64_t PWMOutputPort::get_value(std::string pin_name) {
    auto found = channels.find(pin_name);
    if (found != channels.end()) {
        PWMOutputChannel *channel = dynamic_cast<PWMOutputChannel *>(found->second.get());
        if (channel == nullptr) {
            return 0;
        }
        return channel->get_value();
    }
    else {
        return 0;
    }
}