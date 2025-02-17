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
    if (!gps_rec->waiting(1000)) {
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
        else {
            logger->log_notice("Read GPS OK");
        }
    }
    return true;
}
std::string GPSHatDriver::pretty() {
    std::string str = eros::eros_utility::PrettyUtility::pretty(odom);
    return str;
}
bool GPSHatDriver::process_data(struct gps_data_t* data) {
    nav_msgs::Odometry new_odom;
    bool updated = false;
    if (data->status == STATUS_NO_FIX) {
        logger->log_warn("NO Fix");
    }
    else if (data->status == STATUS_FIX) {
        logger->log_notice("GPS Fix");
        updated = true;
    }
    else if (data->status == STATUS_DGPS_FIX) {
        logger->log_notice("DGPS Fix");
        updated = true;
    }
    if (updated == true) {
        new_odom.header.stamp = eros::eros_utility::ConvertUtility::convert_time(data->online);
        odom = new_odom;
    }

    return true;
}

}  // namespace ros_hats