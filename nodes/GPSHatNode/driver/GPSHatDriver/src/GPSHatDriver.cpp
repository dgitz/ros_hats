#include "GPSHatDriver.h"
namespace ros_hats {
GPSHatDriver::GPSHatDriver() {
}
GPSHatDriver::~GPSHatDriver() {
    delete gps_rec;
}
bool GPSHatDriver::init(eros::Logger* _logger) {
    logger = _logger;

    gps_rec = new gpsmm("localhost", DEFAULT_GPSD_PORT);
    if (gps_rec->stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
        logger->log_error("GPSD Not Running.");
        return false;
    }
    else {
        logger->log_notice("GPSD Is Running.");
    }

    return true;
}
bool GPSHatDriver::update(double dt) {
    struct gps_data_t* newdata;
    if (!gps_rec->waiting(250)) {
        return true;
    }

    if ((newdata = gps_rec->read()) == NULL) {
        logger->log_warn("GPS Read Data Error.");
        return false;
    }
    else {
        bool status = process_data(newdata);
        if (status == false) {
            logger->log_warn("Unable to process GPS Data.");
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
    bool updated = false;
    if (data->status == STATUS_NO_FIX) {
        logger->log_warn("NO Fix");
    }
    else if (data->status == STATUS_FIX) {
        updated = true;
    }
    else if (data->status == STATUS_DGPS_FIX) {
        updated = true;
    }
    if (updated == true) {
        new_gps_data.timestamp = convert_time(data->online);
        new_gps_data.latitude = data->fix.latitude;
        new_gps_data.longitude = data->fix.longitude;
        gps_data = new_gps_data;
    }

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