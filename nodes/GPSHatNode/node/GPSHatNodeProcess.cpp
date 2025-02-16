#include "GPSHatNodeProcess.h"
namespace ros_hats {
GPSHatNodeProcess::GPSHatNodeProcess() {
}
GPSHatNodeProcess::~GPSHatNodeProcess() {
}
eros::eros_diagnostic::Diagnostic GPSHatNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag;
}
void GPSHatNodeProcess::reset() {
}
eros::eros_diagnostic::Diagnostic GPSHatNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<eros::eros_diagnostic::Diagnostic> GPSHatNodeProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros::eros_diagnostic::Diagnostic> GPSHatNodeProcess::check_programvariables() {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string GPSHatNodeProcess::pretty() {
    std::string str = "Node State: " + eros::Node::NodeStateString(get_nodestate());
    return str;
}

}  // namespace ros_hats