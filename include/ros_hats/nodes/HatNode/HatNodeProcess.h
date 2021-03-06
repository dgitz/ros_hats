/*! \file HatNodeProcess.h
 */
#ifndef HatNodeProcess_H
#define HatNodeProcess_H
#include <eros/BaseNodeProcess.h>
#include <ros_hats/ROSHATS_Definitions.h>
/*! \class HatNodeProcess HatNodeProcess.h "HatNodeProcess.h"
 *  \brief */
class HatNodeProcess : public eros::BaseNodeProcess
{
   public:
    HatNodeProcess();
    ~HatNodeProcess();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    static std::string pretty(std::map<std::string, HatConfig> hat_configs);
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    std::map<std::string, HatConfig> load_hat_config(
        std::string file_path = "~/config/DeviceList.json");
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
#endif  // HatNodeProcess_H
