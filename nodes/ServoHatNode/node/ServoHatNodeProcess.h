/**
 * @file ServoHatNodeProcess.h
 * @author David Gitz
 * @brief Servo Hat Node Process for Node
 * @date 2025-02-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <eros_diagnostic/Diagnostic.h>
#include <std_msgs/UInt16.h>

#include "IServoHatDriver.h"
#ifdef ARCHITECTURE_ARMV7L
#include "ServoHatDriver.h"
#else
#include "MockServoHatDriver.h"
#endif
namespace ros_hats {
/*! \class ServoHatNodeProcess ServoHatNodeProcess.h "ServoHatNodeProcess.h"
 *  \brief */
class ServoHatNodeProcess : public eros::BaseNodeProcess
{
   public:
    ServoHatNodeProcess();
    ~ServoHatNodeProcess();
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
    std::map<std::string, IServoHatDriver::ChannelDefinition> get_channels() {
        return driver->get_channels();
    }
    bool new_servo_command(std::string channel_name, std_msgs::UInt16 msg);

   private:
    IServoHatDriver* driver;
    std::map<std::string, IServoHatDriver::ChannelDefinition> channel_map;
};
}  // namespace ros_hats
