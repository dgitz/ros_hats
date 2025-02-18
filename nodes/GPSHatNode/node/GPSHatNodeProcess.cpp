#include "GPSHatNodeProcess.h"
namespace ros_hats {
GPSHatNodeProcess::GPSHatNodeProcess() {
}
GPSHatNodeProcess::~GPSHatNodeProcess() {
    delete logger;
}
eros::eros_diagnostic::Diagnostic GPSHatNodeProcess::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    driver = new GPSHatDriver;
    driver->init(logger);
    return diag;
}

void GPSHatNodeProcess::reset() {
}
eros::eros_diagnostic::Diagnostic GPSHatNodeProcess::update(double t_dt, double t_ros_time) {
    eros::eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    driver->update(t_dt);
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
    str += driver->pretty();
    return str;
}
sensor_msgs::NavSatFix GPSHatNodeProcess::convert(GPSHatDriver::GPSHatDriverContainer hat_output) {
    sensor_msgs::NavSatFix gps_data;
    gps_data.header.stamp = hat_output.timestamp;
    gps_data.header.frame_id = "geographic";
    gps_data.latitude = hat_output.latitude;
    gps_data.longitude = hat_output.longitude;
    return gps_data;
}
sensor_msgs::NavSatFix GPSHatNodeProcess::get_gps_data() {
    return convert(driver->get_gps_data());
}
}  // namespace ros_hats