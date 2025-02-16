/*! \file GPSHatNodeProcess.h
 */
#pragma once
#include <eros/BaseNodeProcess.h>
#include <eros_diagnostic/Diagnostic.h>
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

   private:
};
}  // namespace ros_hats
