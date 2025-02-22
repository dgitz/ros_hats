#include "GPSHatDriver.h"
namespace ros_hats {
GPSHatDriver::GPSHatDriver() {
}
GPSHatDriver::~GPSHatDriver() {
    finish();
}
bool GPSHatDriver::finish() {
    delete gps_rec;
    return true;
}
bool GPSHatDriver::init(eros::Logger* _logger) {
    logger = _logger;

    gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    if (gps_rec->stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
        // GCOVR_EXCL_START
        logger->log_error("GPSD Not Running.");
        return false;
        // GCOVR_EXCL_STOP
    }
    else {
        logger->log_notice("GPSD Is Running.");
    }

    return true;
}
bool GPSHatDriver::update(double dt) {
    struct gps_data_t* newdata;
    if (!gps_rec->waiting((uint64_t)(1000.0 * dt))) {
        return true;
    }

    if ((newdata = gps_rec->read()) == NULL) {
        // GCOVR_EXCL_START
        logger->log_warn("GPS Read Data Error.");
        return false;
        // GCOVR_EXCL_STOP
    }
    else {
        bool status = process_data(newdata);
        if (status == false) {
            // GCOVR_EXCL_START
            logger->log_warn("Unable to process GPS Data.");
            // GCOVR_EXCL_STOP
        }
    }
    return true;
}
std::string GPSHatDriver::pretty() {
    std::string str = GPSHatDriverContainer::pretty(gps_data);
    return str;
}
bool GPSHatDriver::process_data(struct gps_data_t* data) {
    GPSHatDriverContainer new_gps_data;

    new_gps_data.timestamp = convert_time(data->online);
    new_gps_data.geographic_coordinates.latitude_deg = data->fix.latitude;
    new_gps_data.geographic_coordinates.longitude_deg = data->fix.longitude;
    new_gps_data.altitude = data->fix.altitude;
    new_gps_data.latitude_accuracy_m = data->fix.epy;
    new_gps_data.longitude_accuracy_m = data->fix.epx;
    new_gps_data.altitude_accuracy_m = data->fix.epv;
    new_gps_data.course_deg = data->fix.track;
    new_gps_data.course_accuracy_deg = data->fix.epd;
    // GCOVR_EXCL_START
    switch (data->fix.mode) {
        case MODE_NOT_SEEN: new_gps_data.fix_type = FixType::NOT_SEEN; break;
        case MODE_NO_FIX: new_gps_data.fix_type = FixType::NO_FIX; break;
        case MODE_2D: new_gps_data.fix_type = FixType::FIX_2D; break;
        case MODE_3D: new_gps_data.fix_type = FixType::FIX_3D; break;
        default: break;
    }
    switch (data->status) {
        case STATUS_NO_FIX: new_gps_data.status_type = StatusType::NO_FIX; break;
        case STATUS_FIX: new_gps_data.status_type = StatusType::FIX; break;
        case STATUS_DGPS_FIX: new_gps_data.status_type = StatusType::DGPS_FIX; break;
        default: break;
    }
    // GCOVR_EXCL_STOP
    gps_data = new_gps_data;

    return true;
}
#ifdef ARCHITECTURE_ARMV7L
ros::Time GPSHatDriver::convert_time(timestamp_t t_) {
    ros::Time t = eros::eros_utility::ConvertUtility::convert_time((double)t_);
    return t;
}
#else
ros::Time GPSHatDriver::convert_time(timespec t_) {
    ros::Time t;
    t.sec = t_.tv_sec;
    t.nsec = t_.tv_nsec;
    return t;
}
#endif
}  // namespace ros_hats