#pragma once
#include <eros/Logger.h>
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/PrettyUtility.h>
#include <gps.h>
#include <nav_msgs/Odometry.h>

#include "libgpsmm.h"
namespace ros_hats {
class GPSHatDriver
{
   public:
    GPSHatDriver();
    virtual ~GPSHatDriver();
    bool init(eros::Logger* _logger);
    bool update(double dt);
    bool finish();
    bool process_data(struct gps_data_t* data);
    nav_msgs::Odometry get_odom() {
        return odom;
    }
    std::string pretty();

   private:
    eros::Logger* logger;
    gpsmm* gps_rec;
    nav_msgs::Odometry odom;
};
}  // namespace ros_hats