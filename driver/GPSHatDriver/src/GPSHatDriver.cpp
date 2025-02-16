#include "../include/GPSHatDriver.h"
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
bool GPSHatDriver::process_data(struct gps_data_t* data) {
    if (data->status == STATUS_NO_FIX) {
        logger->log_warn("NO Fix");
    }
    else if (data->status == STATUS_FIX) {
        logger->log_notice("GPS Fix");
    }
    else if (data->status == STATUS_DGPS_FIX) {
        logger->log_notice("DGPS Fix");
    }
    return true;
}
}  // namespace ros_hats