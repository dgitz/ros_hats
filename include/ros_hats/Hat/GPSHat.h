/*! \file GPSHat.h
 */
#ifndef ROSHATS_GPSHAT_H
#define ROSHATS_GPSHAT_H
#include <ros_hats/Channel/GPSInputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/GPSInputPort.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include "Driver/GPSHatDriver.h"
/*! \class GPSHat
    \brief GPSHat class
    Basic container for a GPSHat
*/
class GPSHat : public Hat
{
   public:
    enum class HatModel {
        UNKNOWN = 0,  /*!< Uninitialized value. */
        STANDARD = 1, /*!< Standard GPS Hat that is connected via Serial Port.  Uses GPS Daemon to
                         processing. */
        END_OF_LIST = 2, /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert GPSHat::HatModel to human readable string
    /*!
      \param v GPSHat::HatModel type
      \return The converted string.
    */
    static std::string HatModelString(GPSHat::HatModel v) {
        switch (v) {
            case GPSHat::HatModel::UNKNOWN: return "UNKNOWN"; break;
            case GPSHat::HatModel::STANDARD: return "Standard"; break;
            default: return HatModelString(GPSHat::HatModel::UNKNOWN); break;
        }
    }
    static HatModel HatModelType(std::string v) {
        if (v == "Standard") {
            return GPSHat::HatModel::STANDARD;
        }
        else {
            return GPSHat::HatModel::UNKNOWN;
        }
    }
    GPSHat(HatModel _model) : model(_model) {
    }
    ~GPSHat();
    bool init(Logger *_logger, RaspberryPiDefinition::RaspberryPiModel _board, HatConfig _config);
    ChannelDefinition::ChannelErrorType update_pin(std::string pin_name, int64_t value);
    bool init_ros(boost::shared_ptr<ros::NodeHandle>, std::string host_name);
    std::string pretty(std::string pre);
    bool update(double dt);
    std::vector<PortConfig> create_default_port_configs();

    sensor_msgs::NavSatFix convert(GPSInputChannel ch);

    bool cleanup();

   private:
    HatModel model;
    HatConfig hat_config;
    GPSHatDriver *driver;
    GPSInputPort gps_port;
    std::vector<ros::Publisher> gps_navsat_pubs;
};
#endif  // ROSHATS_GPSHAT_H