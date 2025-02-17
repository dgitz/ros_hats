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
    enum class FixType { NO_FIX = 1, FIX = 2, DPGS_FIX = 3 };
    struct GPSHatDriverContainer {
        double latitude;
        double longitude;
        double timestamp;
        FixType fix_type;
    };
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