/*! \file HatNodeProcess.h
 */
#ifndef HatNodeProcess_H
#define HatNodeProcess_H
#include <eros/BaseNodeProcess.h>
/*! \class HatNodeProcess HatNodeProcess.h "HatNodeProcess.h"
 *  \brief */
class HatNodeProcess : public BaseNodeProcess
{
   public:
    HatNodeProcess();
    ~HatNodeProcess();
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        eros::command msg);
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    void cleanup() {
        base_cleanup();
        return;
    }

   private:
};
#endif // HatNodeProcess_H
