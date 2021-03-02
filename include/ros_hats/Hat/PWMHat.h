/*! \file PWMHat.h
 */
#ifndef ROSHATS_PWMHAT_H
#define ROSHATS_PWMHAT_H
#include <ros_hats/Channel/PWMOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/PWMOutputPort.h>
#include <std_msgs/Int64.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
/*! \class PWMHat
    \brief RelayHat class
    Basic container for a RelayHat
*/
class PWMHat : public Hat
{
   public:
    enum class HatModel {
        UNKNOWN = 0,                /*!< Uninitialized value. */
        ADAFRUIT_SERVOHAT_16CH = 1, /*!< Adafruit 16 Channel Servo Hat"  */
        END_OF_LIST = 2,            /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert PWMHat::HatModel to human readable string
    /*!
      \param v PWMHat::HatModel type
      \return The converted string.
    */
    static std::string HatModelString(PWMHat::HatModel v) {
        switch (v) {
            case PWMHat::HatModel::UNKNOWN: return "UNKNOWN"; break;
            case PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH: return "Adafruit 16Ch Servo Hat"; break;
            default: return HatModelString(PWMHat::HatModel::UNKNOWN); break;
        }
    }
    static HatModel HatModelType(std::string v) {
        if (v == "Adafruit 16Ch Servo Hat") {
            return PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH;
        }
        else {
            return PWMHat::HatModel::UNKNOWN;
        }
    }
    PWMHat(HatModel _model) : model(_model) {
    }
    ~PWMHat();
    bool init(Logger *_logger,
              std::string _name_name,
              std::vector<std::string> _pin_names = {},
              std::vector<uint16_t> _pin_numbers = {});
    ChannelDefinition::ChannelErrorType update_pin(std::string port_name,
                                                   std::string pin_name,
                                                   int64_t value);
    bool init_ros(boost::shared_ptr<ros::NodeHandle>, std::string host_name);
    void PWMOutputCallback(const std_msgs::Int64::ConstPtr &msg,
                           const std::string &port_name,
                           const std::string &pin_name);
    std::string pretty();

   private:
    HatModel model;
    std::map<std::string, PWMOutputPort> pwm_ports;
    std::vector<ros::Subscriber> pwmoutput_subs;
};
#endif