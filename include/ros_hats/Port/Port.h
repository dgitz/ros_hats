/*! \file Port.h
 */
#ifndef ROSHATS_PORT_H
#define ROSHATS_PORT_H
#include <ros_hats/Channel/Channel.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

/*! \class Port
    \brief Port class
    Basic container for a Port
*/
class Port
{
   public:
    Port();
    Port(std::string _name,
         std::vector<std::string> _pin_names,
         std::vector<uint16_t> _pin_numbers,
         ChannelDefinition::ChannelType _port_type,
         ChannelDefinition::Direction _direction)
        : name(_name), port_type(_port_type), port_direction(_direction) {
        port_size = (uint16_t)_pin_names.size();
    }
    ~Port();
    bool base_init();
    virtual bool init() = 0;
    uint16_t get_port_size() {
        return port_size;
    }
    std::string get_name() {
        return name;
    }

    std::string base_pretty();
    virtual std::string pretty() = 0;

   protected:
    std::string name;
    uint16_t port_size;
    ChannelDefinition::ChannelType port_type;
    ChannelDefinition::Direction port_direction;
    std::map<std::string, std::shared_ptr<Channel>> channels;
};
#endif  // ROSHATS_PORT_H