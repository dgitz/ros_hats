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
    Port(PortConfig _config) : port_config(_config) {
    }
    ~Port();
    bool base_init();
    virtual bool init() = 0;
    uint16_t get_port_size() {
        return channels.size();
    }
    std::string get_name() {
        return port_config.port_name;
    }

    std::string base_pretty();
    virtual std::string pretty() = 0;

   protected:
    PortConfig port_config;
    std::map<std::string, std::shared_ptr<Channel>> channels;
};
#endif  // ROSHATS_PORT_H