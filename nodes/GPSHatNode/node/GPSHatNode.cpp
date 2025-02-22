#include "GPSHatNode.h"
bool kill_node = false;
namespace ros_hats {

GPSHatNode::GPSHatNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&GPSHatNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
GPSHatNode::~GPSHatNode() {
}
void GPSHatNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    eros::eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
    diag = process->update_diagnostic(
        eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        eros::Level::Type::WARN,
        eros::eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported CommandAction: " +
            eros::Command::CommandString((eros::Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
void GPSHatNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = eros::eros_utility::ConvertUtility::convert_fromptr(t_msg);
    eros::eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    diag = process->update_diagnostic(
        eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS,
        eros::Level::Type::WARN,
        eros::eros_diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " +
            eros::Command::CommandString((eros::Command::Type)cmd.Command));
    logger->log_diagnostic(diag);
}
bool GPSHatNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    eros::Node::State req_state = eros::Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = eros::Node::NodeStateString(process->get_nodestate());
    return true;
}
bool GPSHatNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new GPSHatNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > eros::Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    diagnostic = read_launchparameters();
    if (diagnostic.level > eros::Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }

    process->initialize(get_basenodename(),
                        get_nodename(),
                        get_hostname(),
                        DIAGNOSTIC_SYSTEM,
                        DIAGNOSTIC_SUBSYSTEM,
                        DIAGNOSTIC_COMPONENT,
                        logger);
    std::vector<eros::eros_diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS);
    diagnostic_types.push_back(eros::eros_diagnostic::DiagnosticType::POSE);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
    diagnostic = finish_initialization();
    if (diagnostic.level > eros::Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    if (diagnostic.level < eros::Level::Type::WARN) {
        diagnostic.type = eros::eros_diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = eros::Level::Type::INFO;
        diagnostic.message = eros::eros_diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }
    if (process->request_statechange(eros::Node::State::RUNNING, true) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         eros::Node::NodeStateString(eros::Node::State::RUNNING));
        // LCOV_EXCL_STOP
    }
    logger->log_notice("Node State: " + eros::Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}
eros::eros_diagnostic::Diagnostic GPSHatNode::read_launchparameters() {
    eros::eros_diagnostic::Diagnostic diag = diagnostic;
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &GPSHatNode::command_Callback, this);
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
eros::eros_diagnostic::Diagnostic GPSHatNode::finish_initialization() {
    eros::eros_diagnostic::Diagnostic diag = diagnostic;
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &GPSHatNode::changenodestate_service, this);
    gps_data_pub = n->advertise<sensor_msgs::NavSatFix>(robot_namespace + "/robot/gps_fix", 20);
    gps_pose_pub = n->advertise<nav_msgs::Odometry>(robot_namespace + "/robot/gps_pose", 20);
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::COMMUNICATIONS,
                                      eros::Level::Type::INFO,
                                      eros::eros_diagnostic::Message::NOERROR,
                                      "Comms Ready.");
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::SOFTWARE,
                                      eros::Level::Type::INFO,
                                      eros::eros_diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::DATA_STORAGE,
                                      eros::Level::Type::INFO,
                                      eros::eros_diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::POSE,
                                      eros::Level::Type::WARN,
                                      eros::eros_diagnostic::Message::INITIALIZING,
                                      "GPS Initializing...");
    return diag;
}
bool GPSHatNode::run_loop1() {
    return true;
}
bool GPSHatNode::run_loop2() {
    return true;
}
bool GPSHatNode::run_loop3() {
    return true;
}
bool GPSHatNode::run_001hz() {
    return true;
}
bool GPSHatNode::run_01hz() {
    return true;
}
bool GPSHatNode::run_01hz_noisy() {
    eros::eros_diagnostic::Diagnostic diag = diagnostic;
    logger->log_debug(pretty());
    return true;
}
std::string GPSHatNode::pretty() {
    std::string str = process->pretty();
    return str;
}
bool GPSHatNode::run_1hz() {
    std::vector<eros::eros_diagnostic::Diagnostic> latest_diagnostics =
        process->get_latest_diagnostics();
    for (std::size_t i = 0; i < latest_diagnostics.size(); ++i) {
        logger->log_diagnostic(latest_diagnostics.at(i));
        diagnostic_pub.publish(
            eros::eros_diagnostic::DiagnosticUtility::convert(latest_diagnostics.at(i)));
    }
    eros::eros_diagnostic::Diagnostic diag = process->get_root_diagnostic();
    if (process->get_nodestate() == eros::Node::State::RESET) {
        base_reset();
        process->reset();
        logger->log_notice("Node has Reset");
        if (process->request_statechange(eros::Node::State::RUNNING) == false) {
            // No practical way to unit test
            // LCOV_EXCL_START
            diag = process->update_diagnostic(eros::eros_diagnostic::DiagnosticType::SOFTWARE,
                                              eros::Level::Type::ERROR,
                                              eros::eros_diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
            // LCOV_EXCL_STOP
        }
    }
    return true;
}
bool GPSHatNode::run_10hz() {
    process->update(0.1, ros::Time::now().toSec());
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());

    gps_data_pub.publish(process->get_gps_data());
    gps_pose_pub.publish(process->get_gps_pose_data());
    return true;
}
void GPSHatNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void GPSHatNode::cleanup() {
    process->request_statechange(eros::Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
// No practical way to unit test
}  // namespace ros_hats
// LCOV_EXCL_START
void signalinterrupt_handler(int sig) {
    printf("Killing GPSHatNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
// LCOV_EXCL_STOP

int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "gps_hat_node");
    ros_hats::GPSHatNode *node = new ros_hats::GPSHatNode();
    bool status = node->start();
    if (status == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return EXIT_FAILURE;
        // LCOV_EXCL_STOP
    }
    std::thread thread(&ros_hats::GPSHatNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
