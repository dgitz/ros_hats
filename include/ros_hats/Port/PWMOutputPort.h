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
    PWMOutputPort(std::string _name,
                  std::vector<std::string> _pin_names,
                  std::vector<uint16_t> _pin_numbers)
        : Port(_name,
               _pin_names,
               _pin_numbers,
               ChannelDefinition::ChannelType::PWM,
               ChannelDefinition::Direction::OUTPUT) {
        if (_pin_names.size() != _pin_numbers.size()) {
            return;
        }
        for (uint16_t i = 0; i < port_size; ++i) {
            channels.emplace(std::make_pair(_pin_names.at(i),
                                            new PWMOutputChannel(_name + "_" + std::to_string(i),
                                                                 _pin_names.at(i),
                                                                 _pin_numbers.at(i),
                                                                 1500,
                                                                 1000,
                                                                 2000)));
        }
    }
    ~PWMOutputPort();
    bool init();
    std::string get_name() {
        return name;
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