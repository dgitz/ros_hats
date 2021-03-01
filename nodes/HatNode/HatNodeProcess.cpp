#include "HatNodeProcess.h"

HatNodeProcess::HatNodeProcess() {
}
HatNodeProcess::~HatNodeProcess() {
}
Diagnostic::DiagnosticDefinition HatNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void HatNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition HatNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> HatNodeProcess::new_commandmsg(
    eros::command msg) {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> HatNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
