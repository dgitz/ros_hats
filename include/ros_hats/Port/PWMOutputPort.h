/*! \file PWMOutputPort.h
 */
#ifndef ROSHATS_PWMOUTPUTPORT_H
#define ROSHATS_PWMOUTPUTPORT_H
#include <ros_hats/Channel/PWMOutputChannel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class PWMOutputPort
    \brief PWMOutputPort class
    Basic container for a PWMOutputPort
*/
class PWMOutputPort : public Port
{
   public:
    PWMOutputPort();
    PWMOutputPort(PortConfig _config) : Port(_config) {
        port_config.port_type = ChannelDefinition::ChannelType::PWM;
        for (uint16_t i = 0; i < port_config.channels.size(); ++i) {
            channels.emplace(std::make_pair(port_config.channels.at(i).channel_name,
                                            new PWMOutputChannel(port_config.channels.at(i))));
        }
    }
    ~PWMOutputPort();
    bool init();
    std::string get_name() {
        return port_config.port_name;
    }
    std::vector<PWMOutputChannel> get_channels() {
        std::vector<PWMOutputChannel> _channels;
        for (auto ch : channels) {
            PWMOutputChannel *channel = dynamic_cast<PWMOutputChannel *>(ch.second.get());
            if (channel != nullptr) {
                _channels.push_back(*channel);
            }
        }
        return _channels;
    }
    PWMOutputChannel get_channel(std::string pin_name) {
        PWMOutputChannel empty;
        std::map<std::string, std::shared_ptr<Channel>>::iterator ch_it = channels.find(pin_name);
        if (ch_it == channels.end()) {
            return empty;
        }
        PWMOutputChannel *channel = dynamic_cast<PWMOutputChannel *>(ch_it->second.get());
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
#endif  // ROSHATS_PWMOUTPUTPORT_H