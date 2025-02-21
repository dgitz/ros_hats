#include <ros_hats/Port/DigitalInputPort.h>
DigitalInputPort::DigitalInputPort() {
}
DigitalInputPort::~DigitalInputPort() {
}
std::string DigitalInputPort::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    for (auto ch : channels) { str += ch.second->pretty(pre + "\t") + "\n"; }
    return str;
}
bool DigitalInputPort::init() {
    bool v = base_init();
    return v;
}
ChannelDefinition::ChannelErrorType DigitalInputPort::update(std::string pin_name, int64_t value) {
    auto found = channels.find(pin_name);
    if (found != channels.end()) {
        DigitalInputChannel *channel = dynamic_cast<DigitalInputChannel *>(found->second.get());
        if (channel == nullptr) {
            return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
        }
        return channel->update_value(value);
    }
    else {
        return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    }
}
int64_t DigitalInputPort::get_value(std::string pin_name) {
    auto found = channels.find(pin_name);
    if (found != channels.end()) {
        DigitalInputChannel *channel = dynamic_cast<DigitalInputChannel *>(found->second.get());
        if (channel == nullptr) {
            return 0;
        }
        return channel->get_value();
    }
    else {
        return 0;
    }
}