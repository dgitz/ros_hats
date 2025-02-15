#include <ros_hats/Hat/Driver/GPSHatDriver.h>
GPSHatDriver::GPSHatDriver() : run_time(0.0) {
}
GPSHatDriver::~GPSHatDriver() {
}
bool GPSHatDriver::init(Logger* _logger) {
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
    run_time += dt;
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
    }
    return true;
}
bool GPSHatDriver::finish() {
    delete gps_rec;
    return false;
}
bool GPSHatDriver::process_data(struct gps_data_t* data) {
    position.latitude = data->fix.latitude;
    position.longitude = data->fix.longitude;
    position.altitude = data->fix.altitude;
    position.timestamp = data->online;
    position.update_count++;

    if (data->status == STATUS_NO_FIX) {
        status.fix_type = GPSInputChannel::FixType::NO_FIX;
    }
    else if (data->status == STATUS_FIX) {
        status.fix_type = GPSInputChannel::FixType::FIX_GPS;
    }
    else if (data->status == STATUS_DGPS_FIX) {
        status.fix_type = GPSInputChannel::FixType::FIX_DGPS;
    }

    status.satellites_visible = data->satellites_visible;
    status.timestamp = data->online;
    status.update_count++;
    return true;
}