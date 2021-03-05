/*! \file TerminalHat.h
 */
#ifndef ROSHATS_TERMINALHAT_H
#define ROSHATS_TERMINALHAT_H
#include <ros_hats/Channel/DigitalInputChannel.h>
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/DigitalInputPort.h>
#include <ros_hats/Port/DigitalOutputPort.h>
#include <std_msgs/Int64.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include "Driver/TerminalHatDriver.h"
/*! \class TerminalHat
    \brief TerminalHat class
    Basic container for a TerminalHat
*/
class TerminalHat : public Hat
{
   public:
    enum class HatModel {
        UNKNOWN = 0, /*!< Uninitialized value. */
        GENERIC = 1, /*!< Generic Terminal Hat.  Essentially acts as if no hat is connected."  */
        END_OF_LIST = 2, /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert TerminalHat::HatModel to human readable string
    /*!
      \param v TerminalHat::HatModel type
      \return The converted string.
    */
    static std::string HatModelString(TerminalHat::HatModel v) {
        switch (v) {
            case TerminalHat::HatModel::UNKNOWN: return "UNKNOWN"; break;
            case TerminalHat::HatModel::GENERIC: return "GENERIC"; break;
            default: return HatModelString(TerminalHat::HatModel::UNKNOWN); break;
        }
    }
    static HatModel HatModelType(std::string v) {
        if (v == "GENERIC") {
            return TerminalHat::HatModel::GENERIC;
        }
        else {
            return TerminalHat::HatModel::UNKNOWN;
        }
    }
    TerminalHat(HatModel _model) : model(_model) {
    }
    ~TerminalHat();
    bool init(Logger *_logger, HatConfig _config);
    ChannelDefinition::ChannelErrorType update_pin(std::string port_name,
                                                   std::string pin_name,
                                                   int64_t value);
    bool init_ros(boost::shared_ptr<ros::NodeHandle>, std::string host_name);
    void DigitalOutputCallback(const std_msgs::Int64::ConstPtr &msg,
                               const std::string &port_name,
                               const std::string &pin_name);
    std::string pretty(std::string pre);
    bool cleanup();

   private:
    TerminalHatDriver driver;
    HatModel model;
    HatConfig hat_config;

    std::map<std::string, DigitalOutputPort> digital_output_ports;
    std::map<std::string, DigitalInputPort> digital_input_ports;

    std::vector<ros::Subscriber> digitaloutput_subs;
    std::vector<ros::Publisher> digitalinput_pubs;
};
#endif  // ROSHATS_TERMINALHAT_H