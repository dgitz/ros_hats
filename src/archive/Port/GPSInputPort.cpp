#include <ros_hats/Port/GPSInputPort.h>
GPSInputPort::GPSInputPort() {
}
GPSInputPort::~GPSInputPort() {
}
std::string GPSInputPort::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    for (auto ch : channels) { str += ch.second->pretty(pre + "\t") + "\n"; }
    return str;
}
bool GPSInputPort::init() {
    bool v = base_init();
    return v;
}
ChannelDefinition::ChannelErrorType GPSInputPort::update(std::string channel_name,
                                                         GPSInputChannel::Position _position,
                                                         GPSInputChannel::Status _status) {
    auto found = channels.find(channel_name);
    if (found != channels.end()) {
        GPSInputChannel *channel = dynamic_cast<GPSInputChannel *>(found->second.get());
        if (channel == nullptr) {
            return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
        }
        return channel->update_value(_position, _status);
    }
    else {
        return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    }
}