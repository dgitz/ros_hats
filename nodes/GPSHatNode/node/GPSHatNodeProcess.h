/**
 * @file GPSHatNodeProcess.h
 * @author David Gitz
 * @brief GPS Hat Node Process for Node
 * @date 2025-02-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <eros_diagnostic/Diagnostic.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "GPSHatDriver.h"
namespace ros_hats {
/*! \class GPSHatNodeProcess GPSHatNodeProcess.h "GPSHatNodeProcess.h"
 *  \brief */
class GPSHatNodeProcess : public eros::BaseNodeProcess
{
   public:
    GPSHatNodeProcess();
    ~GPSHatNodeProcess();
    eros::eros_diagnostic::Diagnostic finish_initialization();
    void reset();
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }
    std::string pretty() override;
    sensor_msgs::NavSatFix get_gps_data();
    static sensor_msgs::NavSatFix convert(GPSHatDriver::GPSHatDriverContainer hat_output);

   private:
    GPSHatDriver* driver;
};
}  // namespace ros_hats
