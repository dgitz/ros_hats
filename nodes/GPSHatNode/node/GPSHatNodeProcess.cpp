#include "GPSHatNodeProcess.h"

#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    auto gps_data = driver->get_gps_data();
    latest_nav_sat_fix = convertGPS(gps_data);
    latest_odom = convertPose(gps_data);
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
sensor_msgs::NavSatFix GPSHatNodeProcess::convertGPS(
    GPSHatDriver::GPSHatDriverContainer hat_output) {
    sensor_msgs::NavSatFix gps_data;
    gps_data.header.stamp = hat_output.timestamp;
    gps_data.header.frame_id = "geographic";
    bool gps_data_valid = false;
    switch (hat_output.status_type) {
        case GPSHatDriver::StatusType::UNKNOWN:
            gps_data.status.status =
                -1;  // Collides with gps.h sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            break;
        case GPSHatDriver::StatusType::NO_FIX:
            gps_data.status.status =
                -1;  // Collides with gps.h sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            break;
        case GPSHatDriver::StatusType::FIX:
            gps_data_valid = true;
            gps_data.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;

        case GPSHatDriver::StatusType::DGPS_FIX:
            gps_data_valid = true;
            gps_data.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
    }
    if (gps_data_valid == true) {
        gps_data.latitude = hat_output.geographic_coordinates.latitude_deg;
        gps_data.longitude = hat_output.geographic_coordinates.longitude_deg;
        gps_data.altitude = hat_output.altitude;
        gps_data.position_covariance[0] = hat_output.latitude_accuracy_m;
        gps_data.position_covariance[4] = hat_output.longitude_accuracy_m;
        gps_data.position_covariance[8] = hat_output.altitude_accuracy_m;
        gps_data.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    }
    else {
        gps_data.latitude = 0.0;
        gps_data.longitude = 0.0;
        gps_data.altitude = 0.0;
        gps_data.position_covariance[0] = -1.0;
        gps_data.position_covariance[4] = -1.0;
        gps_data.position_covariance[8] = -1.0;
        gps_data.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
    return gps_data;
}

nav_msgs::Odometry GPSHatNodeProcess::convertPose(GPSHatDriver::GPSHatDriverContainer hat_output) {
    nav_msgs::Odometry odom;
    odom.header.stamp = hat_output.timestamp;
    odom.header.frame_id = "utm";
    UTMCoordinates utm = utm_converter.convert("WGS-84", hat_output.geographic_coordinates);
    odom.pose.pose.position.x = utm.easting_m;
    odom.pose.pose.position.y = utm.northing_m;
    odom.pose.covariance[0] = hat_output.latitude_accuracy_m;
    odom.pose.covariance[7] = hat_output.longitude_accuracy_m;
    odom.pose.covariance[14] = hat_output.altitude_accuracy_m;
    // TODO:  altitude
    double heading_deg = hat_output.course_deg;
    if (isnan(heading_deg)) {
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, 0.0);
        odom.pose.pose.orientation = tf2::toMsg(quat_tf);
    }
    else {
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, hat_output.course_deg * M_PI / 180.0);
        odom.pose.pose.orientation = tf2::toMsg(quat_tf);
    }

    return odom;
}
nav_msgs::Odometry GPSHatNodeProcess::get_gps_pose_data() {
    return latest_odom;
}
sensor_msgs::NavSatFix GPSHatNodeProcess::get_gps_data() {
    sensor_msgs::NavSatFix nav_sat_fix = convertGPS(driver->get_gps_data());
    if ((nav_sat_fix.status.status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) ||
        (nav_sat_fix.status.status == sensor_msgs::NavSatStatus::STATUS_SBAS_FIX)) {
        update_diagnostic(eros::eros_diagnostic::DiagnosticType::POSE,
                          eros::Level::Type::INFO,
                          eros::eros_diagnostic::Message::NOERROR,
                          "GPS Fix Ok");
    }
    else {
        update_diagnostic(eros::eros_diagnostic::DiagnosticType::POSE,
                          eros::Level::Type::WARN,
                          eros::eros_diagnostic::Message::DROPPING_PACKETS,
                          "GPS No Fix");
    }
    return nav_sat_fix;
}
}  // namespace ros_hats