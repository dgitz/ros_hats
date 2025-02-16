/*! \file GPSInputPort.h
 */
#ifndef ROSHATS_GPSINPUTPORT_H
#define ROSHATS_GPSINPUTPORT_H
#include <ros_hats/Channel/GPSInputChannel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class GPSInputPort
    \brief GPSInputPort class
    Basic container for a GPSInputPort
*/
class GPSInputPort : public Port
{
   public:
    GPSInputPort();
    GPSInputPort(PortConfig _config) : Port(_config) {
        port_config.port_type = ChannelDefinition::ChannelType::GPS;
        for (uint16_t i = 0; i < port_config.channels.size(); ++i) {
            channels.emplace(std::make_pair(port_config.channels.at(i).channel_name,
                                            new GPSInputChannel(port_config.channels.at(i))));
        }
    }
    ~GPSInputPort();
    bool init();
    std::string get_name() {
        return port_config.port_name;
    }
    std::vector<GPSInputChannel> get_channels() {
        std::vector<GPSInputChannel> _channels;
        for (auto ch : channels) {
            GPSInputChannel *channel = dynamic_cast<GPSInputChannel *>(ch.second.get());
            if (channel != nullptr) {
                _channels.push_back(*channel);
            }
        }
        return _channels;
    }
    GPSInputChannel get_channel(std::string channel_name) {
        GPSInputChannel empty;
        std::map<std::string, std::shared_ptr<Channel>>::iterator ch_it =
            channels.find(channel_name);
        if (ch_it == channels.end()) {
            return empty;
        }
        GPSInputChannel *channel = dynamic_cast<GPSInputChannel *>(ch_it->second.get());
        if (channel == nullptr) {
            return empty;
        }
        return *channel;
    }
    ChannelDefinition::ChannelErrorType update(std::string channel_name,
                                               GPSInputChannel::Position _position,
                                               GPSInputChannel::Status _status);
    std::string pretty(std::string pre);

   private:
};
#endif  // ROSHATS_GPSINPUTPORT_H