#pragma once
#include <eros/Logger.h>
#include <gps.h>

#include "libgpsmm.h"
namespace ros_hats {
class GPSHatDriver
{
   public:
    GPSHatDriver();
    virtual ~GPSHatDriver();
    bool init(eros::Logger* _logger);
    bool update(double dt);
    bool finish();
    bool process_data(struct gps_data_t* data);

   private:
    eros::Logger* logger;
    gpsmm* gps_rec;
};
}  // namespace ros_hats