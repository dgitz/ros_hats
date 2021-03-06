/*! \file RelayHat.h
 */
#ifndef ROSHATS_RELAYHAT_H
#define ROSHATS_RELAYHAT_H
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/DigitalOutputPort.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
/*! \class RelayHat
    \brief RelayHat class
    Basic container for a RelayHat
*/
class RelayHat : public Hat
{
   public:
    const uint64_t DELAY_GPIOEXPORT_MS = 30000; /*!< How long to allow GPIO sysfs to update. */
    enum class HatModel {
        UNKNOWN = 0,       /*!< Uninitialized value. */
        RPI_RELAY_HAT = 1, /*!< RPi Relay Hat, also known as: "Electronics-Salon RPi Power Relay
                              Board Expansion Module, for Raspberry Pi A+ B+ 2B 3B."  */
        END_OF_LIST = 2,   /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert RelayHat::HatModel to human readable string
    /*!
      \param v RelayHat::HatModel type
      \return The converted string.
    */
    static std::string HatModelString(RelayHat::HatModel v) {
        switch (v) {
            case RelayHat::HatModel::UNKNOWN: return "UNKNOWN"; break;
            case RelayHat::HatModel::RPI_RELAY_HAT: return "RPi Relay Hat"; break;
            default: return HatModelString(RelayHat::HatModel::UNKNOWN); break;
        }
    }
    static HatModel HatModelType(std::string v) {
        if (v == "RPi Relay Hat") {
            return RelayHat::HatModel::RPI_RELAY_HAT;
        }
        else {
            return RelayHat::HatModel::UNKNOWN;
        }
    }
    RelayHat(HatModel _model) : model(_model) {
    }
    ~RelayHat();
    bool init(Logger *_logger, RaspberryPiDefinition::RaspberryPiModel _board, HatConfig _config);
    ChannelDefinition::ChannelErrorType update_pin(std::string pin_name, int64_t value);
    bool init_ros(boost::shared_ptr<ros::NodeHandle>, std::string host_name);
    void DigitalOutputCallback(const std_msgs::Bool::ConstPtr &msg, const std::string &pin_name);
    std::string pretty(std::string pre);
    std::vector<PortConfig> create_default_port_configs();
    bool update(double dt);
    bool cleanup();

   private:
    bool export_gpio(std::string pin_name);
    bool unexport_gpio(std::string pin_name);
    bool setdir_gpio(std::string pin_name, std::string dir);
    bool setvalue_gpio(std::string pin_name, std::string value);
    HatModel model;
    HatConfig hat_config;
    DigitalOutputPort relay_port;
    std::vector<ros::Subscriber> relayoutput_subs;
};
#endif