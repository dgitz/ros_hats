#ifndef GPSHATDRIVER_H
#define GPSHATDRIVER_H
#include <eros/Logger.h>
#include <gps.h>
#include <ros_hats/Channel/GPSInputChannel.h>

#include "libgpsmm.h"
class GPSHatDriver
{
   public:
    GPSHatDriver();

    ~GPSHatDriver();
    bool init(Logger* _logger);
    bool update(double dt);
    bool finish();

    GPSInputChannel::Position get_position() {
        return position;
    }
    GPSInputChannel::Status get_status() {
        return status;
    }
    bool process_data(struct gps_data_t* data);

   private:
    double run_time;
    Logger* logger;
    gpsmm* gps_rec;
    GPSInputChannel::Position position;
    GPSInputChannel::Status status;
};

#endif  // GPSHATDRIVER_H