/*! \file TerminalHat.h
 */
/* TODO
- GPIO Driver export,set value, read value, unexport
- GPIO Driver removes "GPIO" from pin names
- Digital Input uses Edge callback
- Terminal Hat has ROS Publishers and Subscribers
*/
#ifndef ROSHATS_TERMINALHAT_H
#define ROSHATS_TERMINALHAT_H
#include <ros_hats/Channel/DigitalInputChannel.h>
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/DigitalInputPort.h>
#include <ros_hats/Port/DigitalOutputPort.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>

#include <fstream>
#include <iostream>
#include <string>

/* todo
- document max number of callbacks
- document usage of inputs (only publish when input edge goes to high)
- usage instructions for class (only 1 instance of TerminalHat on a DEVICE!!!)
- force limit of 1 digital input port -- done
- set value of digital input -- done
- test multiple input -- done
- digital output -- done
- get type of raspberry pi -- done
- cleanup pins when done -- done
- memory corruption error when finished
- add US to make RelayHat use wiringPI instead of sysfs
*/
/*! \class TerminalHat
    \brief TerminalHat class
    Basic container for a TerminalHat
*/
std::vector<ros::Publisher>
    ROSHATS_TERMINALHAT_H_digitalinput_pubs;  // Avoid potential global variable conflicts
DigitalInputPort ROSHATS_TERMINALHAT_H_digitalinput_port;
std::vector<std::string> ROSHATS_TERMINALHAT_H_digitalinput_channel_names;
class TerminalHat : public Hat
{
   public:
    const int MAXDIGITALINPUT_CALLBACKS = 10;
    enum class HatModel {
        UNKNOWN = 0,     /*!< Uninitialized value. */
        STANDARD = 1,    /*!< Standard Terminal Hat.  */
        END_OF_LIST = 2, /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert RelayHat::HatModel to human readable string
    /*!
      \param v RelayHat::HatModel type
      \return The converted string.
    */
    static std::string HatModelString(TerminalHat::HatModel v) {
        switch (v) {
            case TerminalHat::HatModel::UNKNOWN: return "UNKNOWN"; break;
            case TerminalHat::HatModel::STANDARD: return "Standard"; break;
            default: return HatModelString(TerminalHat::HatModel::UNKNOWN); break;
        }
    }
    static HatModel HatModelType(std::string v) {
        if (v == "Standard") {
            return TerminalHat::HatModel::STANDARD;
        }
        else {
            return TerminalHat::HatModel::UNKNOWN;
        }
    }
    TerminalHat(HatModel _model) : model(_model) {
    }
    ~TerminalHat();
    void DigitalOutputCallback(const std_msgs::Bool::ConstPtr &msg, const std::string &pin_name);
    bool init(Logger *_logger, RaspberryPiDefinition::RaspberryPiModel _board, HatConfig _config);
    ChannelDefinition::ChannelErrorType update_pin(std::string pin_name, int64_t value);
    bool init_ros(boost::shared_ptr<ros::NodeHandle>, std::string host_name);
    std::string pretty(std::string pre);
    bool update(double dt);
    RaspberryPiDefinition::RaspberryPi get_board() {
        return pi_model;
    }
    bool cleanup();

   private:
    static void update_digitalinput_frompin(uint16_t index);
    static void digitalInputCB_1();
    static void digitalInputCB_2();
    static void digitalInputCB_3();
    static void digitalInputCB_4();
    static void digitalInputCB_5();
    static void digitalInputCB_6();
    static void digitalInputCB_7();
    static void digitalInputCB_8();
    static void digitalInputCB_9();
    static void digitalInputCB_10();
    HatModel model;
    HatConfig hat_config;
    std::vector<ros::Subscriber> digitaloutput_subs;
    DigitalOutputPort digitaloutput_port;
};
#endif  // ROSHATS_TERMINALHAT_H