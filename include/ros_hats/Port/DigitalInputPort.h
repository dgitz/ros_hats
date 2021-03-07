/*! \file DigitalInputPort.h
 */
#ifndef ROSHATS_DIGITALINPUTPORT_H
#define ROSHATS_DIGITALINPUTPORT_H
#include <ros_hats/Channel/DigitalInputChannel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class DigitalInputPort
    \brief DigitalInputPort class
    Basic container for a DigitalInputPort
*/
class DigitalInputPort : public Port
{
   public:
    DigitalInputPort();
    DigitalInputPort(PortConfig _config) : Port(_config) {
        port_config.port_type = ChannelDefinition::ChannelType::DIGITAL;

        for (uint16_t i = 0; i < port_config.channels.size(); ++i) {
            channels.emplace(std::make_pair(port_config.channels.at(i).channel_name,
                                            new DigitalInputChannel(port_config.channels.at(i))));
        }
    }
    ~DigitalInputPort();
    bool init();
    std::vector<DigitalInputChannel> get_channels() {
        std::vector<DigitalInputChannel> _channels;
        for (auto ch : channels) {
            DigitalInputChannel *channel = dynamic_cast<DigitalInputChannel *>(ch.second.get());
            if (channel != nullptr) {
                _channels.push_back(*channel);
            }
        }
        return _channels;
    }
    DigitalInputChannel get_channel(std::string pin_name) {
        DigitalInputChannel empty;
        std::map<std::string, std::shared_ptr<Channel>>::iterator ch_it = channels.find(pin_name);
        if (ch_it == channels.end()) {
            return empty;
        }
        DigitalInputChannel *channel = dynamic_cast<DigitalInputChannel *>(ch_it->second.get());
        if (channel == nullptr) {
            return empty;
        }
        return *channel;
    }
    ChannelDefinition::ChannelErrorType update(std::string pin_name, int64_t value);
    int64_t get_value(std::string pin_name);
    std::string pretty(std::string pre);

   private:
};
#endif  // ROSHATS_DIGITALINPUTPORT_H