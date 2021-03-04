/*! \file HatNodeProcess.h
 */
#ifndef HatNodeProcess_H
#define HatNodeProcess_H
#include <eros/BaseNodeProcess.h>
#include <ros_hats/ROSHATS_Definitions.h>

/*! \class HatNodeProcess HatNodeProcess.h "HatNodeProcess.h"
 *  \brief */
class HatNodeProcess : public BaseNodeProcess
{
   public:
    HatNodeProcess();
    ~HatNodeProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    static std::string pretty(std::map<std::string, HatConfig> hat_configs);
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    std::map<std::string, HatConfig> load_hat_config(
        std::string file_path = "/home/robot/config/DeviceList.json");
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
#endif  // HatNodeProcess_H
