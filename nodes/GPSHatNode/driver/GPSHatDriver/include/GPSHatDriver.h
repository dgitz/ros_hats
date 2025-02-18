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
    enum class FixType { UNKNOWN = 0, NO_FIX = 1, FIX = 2, DPGS_FIX = 3, END_OF_LIST = 4 };
    struct GPSHatDriverContainer {
        ros::Time timestamp;
        double latitude{0.0};
        double longitude{0.0};
        FixType fix_type{FixType::UNKNOWN};
        static std::string pretty(GPSHatDriverContainer data) {
            std::string str;
            str = "GPS: T=" + std::to_string(data.timestamp.toSec());
            str += " Lat: " + std::to_string(data.latitude) +
                   " (Deg) Long: " + std::to_string(data.longitude) + " (Deg)";
            return str;
        }
    };
    GPSHatDriver();
    virtual ~GPSHatDriver();
    bool init(eros::Logger* logger);
    bool update(double dt);
    bool finish();
    bool process_data(struct gps_data_t* data);
    GPSHatDriverContainer get_gps_data() {
        return gps_data;
    }
    std::string pretty();
#ifdef ARCHITECTURE_ARMV7L
    static ros::Time convert_time(timestamp_t t);
#else
    static ros::Time convert_time(timespec t);
#endif

   private:
    eros::Logger* logger;
    gpsmm* gps_rec;
    GPSHatDriverContainer gps_data;
};
}  // namespace ros_hats