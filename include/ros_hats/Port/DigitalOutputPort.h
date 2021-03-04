/*! \file DigitalOutputPort.h
 */
#ifndef ROSHATS_DIGITALOUTPUTPORT_H
#define ROSHATS_DIGITALOUTPUTPORT_H
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class DigitalOutputPort
    \brief DigitalOutputPort class
    Basic container for a DigitalOutputPort
*/
class DigitalOutputPort : public Port
{
   public:
    DigitalOutputPort();
    DigitalOutputPort(PortConfig _config) : Port(_config) {
        port_config.port_type = ChannelDefinition::ChannelType::DIGITAL;

        for (uint16_t i = 0; i < port_config.channels.size(); ++i) {
            channels.emplace(std::make_pair(port_config.channels.at(i).channel_name,
                                            new DigitalOutputChannel(port_config.channels.at(i))));
            /*
            auto ch = std::make_shared<DigitalOutputChannel>(
                DigitalOutputChannel(port_config.channels.at(i)));
            channels.insert(std::pair<std::string, std::shared_ptr<Channel>>(
                port_config.channels.at(i).channel_name, ch));
                */
        }
    }
    ~DigitalOutputPort();
    bool init();
    std::vector<DigitalOutputChannel> get_channels() {
        std::vector<DigitalOutputChannel> _channels;
        for (auto ch : channels) {
            DigitalOutputChannel *channel = dynamic_cast<DigitalOutputChannel *>(ch.second.get());
            if (channel != nullptr) {
                _channels.push_back(*channel);
            }
        }
        return _channels;
    }
    DigitalOutputChannel get_channel(std::string pin_name) {
        DigitalOutputChannel empty;
        std::map<std::string, std::shared_ptr<Channel>>::iterator ch_it = channels.find(pin_name);
        if (ch_it == channels.end()) {
            return empty;
        }
        DigitalOutputChannel *channel = dynamic_cast<DigitalOutputChannel *>(ch_it->second.get());
        if (channel == nullptr) {
            return empty;
        }
        return *channel;
    }
    ChannelDefinition::ChannelErrorType update(std::string pin_name, int64_t value);
    int64_t get_value(std::string pin_name);
    std::string pretty();

   private:
};
#endif  // ROSHATS_DIGITALOUTPUTPORT_H