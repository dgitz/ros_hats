/*! \file ServoOutputPort.h
 */
#ifndef ROSHATS_SERVOOUTPUTPORT_H
#define ROSHATS_SERVOOUTPUTPORT_H
#include <ros_hats/Channel/ServoOutputChannel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class ServoOutputPort
    \brief ServoOutputPort class
    Basic container for a ServoOutputPort
*/
class ServoOutputPort : public Port
{
   public:
    ServoOutputPort();
    ServoOutputPort(PortConfig _config) : Port(_config) {
        port_config.port_type = ChannelDefinition::ChannelType::SERVO;
        for (uint16_t i = 0; i < port_config.channels.size(); ++i) {
            channels.emplace(std::make_pair(port_config.channels.at(i).channel_name,
                                            new ServoOutputChannel(port_config.channels.at(i))));
        }
    }
    ~ServoOutputPort();
    bool init();
    std::string get_name() {
        return port_config.port_name;
    }
    std::vector<ServoOutputChannel> get_channels() {
        std::vector<ServoOutputChannel> _channels;
        for (auto ch : channels) {
            ServoOutputChannel *channel = dynamic_cast<ServoOutputChannel *>(ch.second.get());
            if (channel != nullptr) {
                _channels.push_back(*channel);
            }
        }
        return _channels;
    }
    ServoOutputChannel get_channel(std::string pin_name) {
        ServoOutputChannel empty;
        std::map<std::string, std::shared_ptr<Channel>>::iterator ch_it = channels.find(pin_name);
        if (ch_it == channels.end()) {
            return empty;
        }
        ServoOutputChannel *channel = dynamic_cast<ServoOutputChannel *>(ch_it->second.get());
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
#endif  // ROSHATS_SERVOOUTPUTPORT_H