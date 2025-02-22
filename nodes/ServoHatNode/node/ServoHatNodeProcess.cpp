#include "ServoHatNodeProcess.h"

namespace ros_hats {
ServoHatNodeProcess::ServoHatNodeProcess() {
}
ServoHatNodeProcess::~ServoHatNodeProcess() {
    delete logger;
}
eros::eros_diagnostic::Diagnostic ServoHatNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    // driver = new GPSHatDriver;
    driver->init(logger);
    return diag;
}

void ServoHatNodeProcess::reset() {
}
eros::eros_diagnostic::Diagnostic ServoHatNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    driver->update(t_dt);
    return diag;
}
std::vector<eros::eros_diagnostic::Diagnostic> ServoHatNodeProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros::eros_diagnostic::Diagnostic> ServoHatNodeProcess::check_programvariables() {
    std::vector<eros::eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
std::string ServoHatNodeProcess::pretty() {
    std::string str = "Node State: " + eros::Node::NodeStateString(get_nodestate());
    str += driver->pretty();
    return str;
}
}  // namespace ros_hats