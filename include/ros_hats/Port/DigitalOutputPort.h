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
    DigitalOutputPort(std::string _name,
                      std::vector<std::string> _pin_names,
                      std::vector<uint16_t> _pin_numbers,
                      int64_t _default_value,
                      int64_t _lower_range,
                      int64_t _upper_range)
        : Port(_name,
               _pin_names,
               _pin_numbers,
               ChannelDefinition::ChannelType::DIGITAL,
               ChannelDefinition::Direction::OUTPUT) {
        if (_pin_names.size() != _pin_numbers.size()) {
            return;
        }
        for (uint16_t i = 0; i < port_size; ++i) {
            channels.emplace(
                std::make_pair(_pin_names.at(i),
                               new DigitalOutputChannel(_name + "_" + std::to_string(i),
                                                        _pin_names.at(i),
                                                        _pin_numbers.at(i),
                                                        _default_value,
                                                        _lower_range,
                                                        _upper_range)));
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
    ChannelDefinition::ChannelErrorType update(std::string pin_name, int64_t value);
    int64_t get_value(std::string pin_name);
    std::string pretty();

   private:
};
#endif  // ROSHATS_DIGITALOUTPUTPORT_H