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
    enum class StatusType { UNKNOWN = 0, NO_FIX = 1, FIX = 2, DGPS_FIX = 3, END_OF_LIST = 4 };
    static std::string StatusTypeString(StatusType type) {
        switch (type) {
            case StatusType::UNKNOWN: return "UNKNOWN";
            case StatusType::NO_FIX: return "NO FIX";
            case StatusType::FIX: return "FIX";
            case StatusType::DGPS_FIX: return "DGPS FIX";
            default: return "UNKNOWN";
        }
    }
    enum class FixType {
        UNKNOWN = 0,
        NOT_SEEN = 1,
        NO_FIX = 2,
        FIX_2D = 3,
        FIX_3D = 4,
        END_OF_LIST = 5
    };
    static std::string FixTypeString(FixType type) {
        switch (type) {
            case FixType::UNKNOWN: return "UNKNOWN";
            case FixType::NOT_SEEN: return "NOT SEEN";
            case FixType::NO_FIX: return "NO FIX";
            case FixType::FIX_2D: return "FIX 2D";
            case FixType::FIX_3D: return "FIX 3D";
            default: return "UNKNOWN";
        }
    }
    struct GPSHatDriverContainer {
        ros::Time timestamp;
        double latitude{0.0};
        double longitude{0.0};
        double altitude{0.0};
        double latitude_accuracy_m{-1.0};
        double longitude_accuracy_m{-1.0};
        double altitude_accuracy_m{-1.0};
        FixType fix_type{FixType::UNKNOWN};
        StatusType status_type{StatusType::UNKNOWN};
        static std::string pretty(GPSHatDriverContainer data) {
            std::string str;
            str = "GPS: T=" + std::to_string(data.timestamp.toSec());
            str += " Status: " + StatusTypeString(data.status_type);
            str += " Fix: " + FixTypeString(data.fix_type);
            str += " Lat: " + std::to_string(data.latitude) +
                   " (Deg) Long: " + std::to_string(data.longitude) + " (Deg)";
            return str;
        }
    };
    GPSHatDriver();
    virtual ~GPSHatDriver();
    bool init(eros::Logger* logger);
    bool update(double dt);
    eros::Level::Type get_status() {
        switch (gps_data.status_type) {
            case StatusType::UNKNOWN: return eros::Level::Type::ERROR;
            case StatusType::NO_FIX: return eros::Level::Type::WARN;
            case StatusType::FIX: return eros::Level::Type::NOTICE;
            case StatusType::DGPS_FIX: return eros::Level::Type::NOTICE;
            default: return eros::Level::Type::ERROR;
        }
    }
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
};  // namespace ros_hats
}  // namespace ros_hats