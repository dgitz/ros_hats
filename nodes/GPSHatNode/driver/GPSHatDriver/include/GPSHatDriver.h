/**
 * @file GPSHatDriver.h
 * @author David Gitz
 * @brief
 * @date 2025-02-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/Logger.h>
#include <eros_utility/ConvertUtility.h>
#include <eros_utility/PrettyUtility.h>
#include <gps.h>
#include <ros_hats/CoordinateDefinitions.h>

#include "libgpsmm.h"
//! ros_hats Namespace
namespace ros_hats {
/**
 * @brief GPSHatDriver Class
 * @details Connects to an instance of the gpsd daemon
 *
 */
class GPSHatDriver
{
   public:
    /**
     * @brief GPS Status
     *
     */
    enum class StatusType {
        UNKNOWN = 0,    /**< Unknown Status */
        NO_FIX = 1,     /**< No Fix */
        FIX = 2,        /**< GPS Fix, but only regular GPS Mode */
        DGPS_FIX = 3,   /**< Differential GPS Fix */
        END_OF_LIST = 4 /*!< Last item of list. Used for Range Checks. */
    };
    /**
     * @brief Convert Status Type to string
     *
     * @param type
     * @return std::string
     */
    static std::string StatusTypeString(StatusType type) {
        switch (type) {
            case StatusType::UNKNOWN: return "UNKNOWN";
            case StatusType::NO_FIX: return "NO FIX";
            case StatusType::FIX: return "FIX";
            case StatusType::DGPS_FIX: return "DGPS FIX";
            default: return "UNKNOWN";
        }
    }
    /**
     * @brief GPS Fix Type
     *
     */
    enum class FixType {
        UNKNOWN = 0,    /**< Unknown Fix Type */
        NOT_SEEN = 1,   /**< GPS Not Seen */
        NO_FIX = 2,     /**< No Fix */
        FIX_2D = 3,     /**< 2D Fix Only */
        FIX_3D = 4,     /**< 3D Fix */
        END_OF_LIST = 5 /*!< Last item of list. Used for Range Checks. */
    };
    /**
     * @brief Convert Fix Type to String
     *
     * @param type
     * @return std::string
     */
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
    /**
     * @brief Container for housing full output of GPSHatDriver
     *
     */
    struct GPSHatDriverContainer {
        ros::Time timestamp;
        GeograpicCoordinates geographic_coordinates;
        double altitude{0.0};
        double latitude_accuracy_m{-1.0};
        double longitude_accuracy_m{-1.0};
        double altitude_accuracy_m{-1.0};
        double course_deg{0.0};
        double course_accuracy_deg{-1.0};
        FixType fix_type{FixType::UNKNOWN};
        StatusType status_type{StatusType::UNKNOWN};
        static std::string pretty(GPSHatDriverContainer data) {
            std::string str;
            str = "GPS: T=" + std::to_string(data.timestamp.toSec());
            str += " Status: " + StatusTypeString(data.status_type);
            str += " Fix: " + FixTypeString(data.fix_type);
            str += " Lat: " + std::to_string(data.geographic_coordinates.latitude_deg) +
                   " (Deg) Long: " + std::to_string(data.geographic_coordinates.longitude_deg) +
                   " (Deg)";
            return str;
        }
    };
    GPSHatDriver();
    virtual ~GPSHatDriver();
    /**
     * @brief Initialize GPS Hat Driver
     *
     * @param logger
     * @return true
     * @return false
     */
    bool init(eros::Logger* logger);
    /**
     * @brief Update GPSHatDriver
     * @details Will poll gpsd at this interval.
     *
     * @param dt Delta Time in seconds.
     * @return true
     * @return false
     */
    bool update(double dt);
    /**
     * @brief  Convert GPS Status Type to a common Level
     *
     * @param type
     * @return eros::Level::Type
     */
    static eros::Level::Type get_level(StatusType type) {
        switch (type) {
            case StatusType::UNKNOWN: return eros::Level::Type::ERROR;
            case StatusType::NO_FIX: return eros::Level::Type::WARN;
            case StatusType::FIX: return eros::Level::Type::NOTICE;
            case StatusType::DGPS_FIX: return eros::Level::Type::NOTICE;
            default: return eros::Level::Type::ERROR;
        }
    }
    /**
     * @brief Finish and Close Driver
     *
     * @return true
     * @return false
     */
    bool finish();
    /**
     * @brief Process the incoming GPS Data
     *
     * @param data
     * @return true
     * @return false
     */
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