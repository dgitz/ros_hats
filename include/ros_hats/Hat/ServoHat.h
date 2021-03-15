/*! \file ServoHat.h
 */
#ifndef ROSHATS_SERVOHAT_H
#define ROSHATS_SERVOHAT_H
#include <ros_hats/Hat/Driver/Adafruit16ChServoHat.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/ServoOutputPort.h>
#include <std_msgs/Int64.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
/*! \class ServoHat
    \brief ServoHat class
    Basic container for a ServoHat
*/
class ServoHat : public Hat
{
   public:
    enum class HatModel {
        UNKNOWN = 0,                /*!< Uninitialized value. */
        ADAFRUIT_SERVOHAT_16CH = 1, /*!< Adafruit 16 Channel Servo Hat"  */
        END_OF_LIST = 2,            /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert ServoHat::HatModel to human readable string
    /*!
      \param v ServoHat::HatModel type
      \return The converted string.
    */
    static std::string HatModelString(ServoHat::HatModel v) {
        switch (v) {
            case ServoHat::HatModel::UNKNOWN: return "UNKNOWN"; break;
            case ServoHat::HatModel::ADAFRUIT_SERVOHAT_16CH:
                return "Adafruit 16Ch Servo Hat";
                break;
            default: return HatModelString(ServoHat::HatModel::UNKNOWN); break;
        }
    }
    static HatModel HatModelType(std::string v) {
        if (v == "Adafruit 16Ch Servo Hat") {
            return ServoHat::HatModel::ADAFRUIT_SERVOHAT_16CH;
        }
        else {
            return ServoHat::HatModel::UNKNOWN;
        }
    }
    ServoHat(HatModel _model) : model(_model) {
    }
    ~ServoHat();
    bool init(Logger *_logger, RaspberryPiDefinition::RaspberryPiModel _board, HatConfig _config);
    ChannelDefinition::ChannelErrorType update_pin(std::string port_name,
                                                   std::string pin_name,
                                                   int64_t value);
    bool init_ros(boost::shared_ptr<ros::NodeHandle>, std::string host_name);
    void ServoOutputCallback(const std_msgs::Int64::ConstPtr &msg,
                             const std::string &port_name,
                             const std::string &pin_name);
    std::string pretty(std::string pre);
    std::vector<PortConfig> create_default_port_configs();
    bool update(double dt);
    bool cleanup();

   private:
    Adafruit16ChServoHat driver;
    HatModel model;
    HatConfig hat_config;
    std::map<std::string, ServoOutputPort> servo_ports;
    std::vector<ros::Subscriber> servooutput_subs;
};
#endif  // ROSHATS_SERVOHAT_H