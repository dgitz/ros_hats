#include "HatNode.h"
bool kill_node = false;
HatNode::HatNode()
    : system_command_action_server(
          *n.get(),
          get_hostname() + "_" + HatNode::BASE_NODE_NAME + "_SystemCommand",
          boost::bind(&HatNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
HatNode::~HatNode() {
}
void HatNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
}
void HatNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    (void)t_msg;
}
bool HatNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                      eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool HatNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new HatNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    diagnostic = read_launchparameters();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }

    process->initialize(get_basenodename(),
                        get_nodename(),
                        get_hostname(),
                        DIAGNOSTIC_SYSTEM,
                        DIAGNOSTIC_SUBSYSTEM,
                        DIAGNOSTIC_COMPONENT,
                        logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
    diagnostic = finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    if (diagnostic.level < Level::Type::WARN) {
        diagnostic.type = Diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = Level::Type::INFO;
        diagnostic.message = Diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }
    if (process->request_statechange(Node::State::INITIALIZED) == false) {
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZED));
    }
    if (process->request_statechange(Node::State::RUNNING) == false) {
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::RUNNING));
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}
Diagnostic::DiagnosticDefinition HatNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition HatNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_nodestate_topic = "/" + node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &HatNode::changenodestate_service, this);
    // Read Hat Configuration
    uint16_t index = 0;
    bool hat_search = true;
    std::vector<hat_config> hat_configs;
    while (hat_search == true) {
        std::string hat_name;
        std::string hat_type;
        std::string hat_model;
        {
            char param_str[512];
            sprintf(param_str, "/%s/Hat%03d_name", node_name.c_str(), index);
            if (n->getParam(param_str, hat_name) == false) {
                hat_search = false;
                break;
            }
        }
        {
            char param_str[512];
            sprintf(param_str, "/%s/Hat%03d_type", node_name.c_str(), index);
            if (n->getParam(param_str, hat_type) == false) {
                hat_search = false;
                break;
            }
        }
        {
            char param_str[512];
            sprintf(param_str, "/%s/Hat%03d_model", node_name.c_str(), index);
            if (n->getParam(param_str, hat_model) == false) {
                hat_search = false;
                break;
            }
        }
        hat_config config;
        config.hat_name = hat_name;
        config.hat_type = hat_type;
        config.hat_model = hat_model;
        hat_configs.push_back(config);
        index++;
    }
    // Create Hats
    for (std::size_t i = 0; i < hat_configs.size(); ++i) {
        if (hat_configs.at(i).hat_type == "RelayHat") {
            RelayHat::HatModel model = RelayHat::HatModelType(hat_configs.at(i).hat_model);
            if (model == RelayHat::HatModel::UNKNOWN) {
                diag = process->update_diagnostic(
                    Diagnostic::DiagnosticType::DATA_STORAGE,
                    Level::Type::ERROR,
                    Diagnostic::Message::INITIALIZING_ERROR,
                    "Hat Model: " + hat_configs.at(i).hat_model + " Not Supported.");
                logger->log_diagnostic(diag);
                return diag;
            }
            else {
                hats.emplace(std::make_pair(hat_configs.at(i).hat_name, new RelayHat(model)));
            }
        }
        else {
            diag = process->update_diagnostic(
                Diagnostic::DiagnosticType::DATA_STORAGE,
                Level::Type::ERROR,
                Diagnostic::Message::INITIALIZING_ERROR,
                "Hat Type: " + hat_configs.at(i).hat_type + " Not Supported.");
            logger->log_diagnostic(diag);
            return diag;
        }
    }
    if (hat_configs.size() == 0) {
        diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          Diagnostic::Message::INITIALIZING_ERROR,
                                          "No Hats Defined. Exiting.");
        logger->log_diagnostic(diag);
        return diag;
    }
    for (auto hat_it : hats) {
        RelayHat *hat = dynamic_cast<RelayHat *>(hat_it.second.get());
        if (hat != nullptr) {
            if (hat->init(logger, hat_it.first) == false) {
                diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                                  Level::Type::ERROR,
                                                  Diagnostic::Message::INITIALIZING_ERROR,
                                                  "Unable to initialize Hat: " + hat_it.first);
                return diag;
            }
        }
        else {
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                              Level::Type::ERROR,
                                              Diagnostic::Message::INITIALIZING_ERROR,
                                              "Hat has no memory: " + hat_it.first);
            return diag;
        }
        // Hat Initialized OK.  Now need to setup ROS for the Hat
        if (hat->init_ros(n, host_name) == false) {
            diag = process->update_diagnostic(
                Diagnostic::DiagnosticType::DATA_STORAGE,
                Level::Type::ERROR,
                Diagnostic::Message::INITIALIZING_ERROR,
                "Unable to initialize Hat ROS Connection: " + hat_it.first);
            return diag;
        }
        else {
            logger->log_notice("Hat: " + hat_it.first + " Initialized.");
        }
    }

    diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    return diag;
}
bool HatNode::run_loop1() {
    return true;
}
bool HatNode::run_loop2() {
    return true;
}
bool HatNode::run_loop3() {
    return true;
}
bool HatNode::run_001hz() {
    return true;
}
bool HatNode::run_01hz() {
    return true;
}
bool HatNode::run_01hz_noisy() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    if ((deviceInfo.received == false) && (disable_device_client == false)) {
        logger->log_notice("Requesting Device Info");
        std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
        ros::ServiceClient client = n->serviceClient<eros::srv_device>(device_topic);
        eros::srv_device srv;
        if (client.call(srv)) {
            deviceInfo.received = true;
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                              Level::Type::INFO,
                                              Diagnostic::Message::NOERROR,
                                              "Device Info Received.");
            logger->log_diagnostic(diag);
        }
        else {
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                              Level::Type::WARN,
                                              Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Device Info not received yet.");
            logger->log_diagnostic(diag);
        }
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool HatNode::run_1hz() {
    std::vector<Diagnostic::DiagnosticDefinition> latest_diagnostics =
        process->get_latest_diagnostics();
    for (std::size_t i = 0; i < latest_diagnostics.size(); ++i) {
        logger->log_diagnostic(latest_diagnostics.at(i));
        diagnostic_pub.publish(process->convert(latest_diagnostics.at(i)));
    }
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    if (process->get_nodestate() == Node::State::RESET) {
        process->reset();
        logger->log_notice("Node has Reset");
        if (process->request_statechange(Node::State::RUNNING) == false) {
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                              Level::Type::ERROR,
                                              Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
        }
    }
    return true;
}
bool HatNode::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    return true;
}
void HatNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void HatNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing HatNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "hat_node");
    HatNode *node = new HatNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&HatNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
