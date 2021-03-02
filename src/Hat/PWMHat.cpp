#include <ros_hats/Hat/PWMHat.h>
PWMHat::~PWMHat() {
}
std::string PWMHat::pretty() {
    std::string str = base_pretty();
    str += " Type: PWMHat Model: " + HatModelString(model) + " --- \n";

    if (pwm_ports.size() == 0) {
        str += "  NO Ports Defined.\n";
    }
    else {
        uint16_t counter = 0;
        for (auto port : pwm_ports) {
            str += "  Port: " + std::to_string(counter);
            str += port.second.pretty();
            counter++;
        }
    }
    return str;
}
bool PWMHat::init(Logger *_logger,
                  std::string _name,
                  std::vector<std::string> _pin_names,
                  std::vector<uint16_t> _pin_numbers) {
    bool v = base_init(_logger);
    if (v == false) {
        return false;
    }

    if ((model == HatModel::UNKNOWN) || (model == HatModel::END_OF_LIST)) {
        return false;
    }
    diagnostic.device_name = _name;
    diagnostic.node_name = _name;
    diagnostic.system = System::MainSystem::ROVER;
    diagnostic.subsystem = System::SubSystem::ROBOT_CONTROLLER;
    diagnostic.component = System::Component::GPIO;
    diagnostic.type = Diagnostic::DiagnosticType::ACTUATORS;
    diagnostic.message = Diagnostic::Message::INITIALIZING;
    diagnostic.level = Level::Type::INFO;
    diagnostic.description = "Hat Initializing";
    diag_helper.initialize(diagnostic);
    diag_helper.enable_diagnostics(std::vector<Diagnostic::DiagnosticType>{
        Diagnostic::DiagnosticType::ACTUATORS, Diagnostic::DiagnosticType::COMMUNICATIONS});
    diagnostic = diag_helper.update_diagnostic(diagnostic);
    name = _name;
    if (model == PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH) {
        if ((_pin_names.size() != 0) || (_pin_numbers.size() != 0)) {
            logger->log_warn("Model " + PWMHat::HatModelString(model) +
                             " Does not support overriding pin names or numbers.  Ignoring pin "
                             "names and numbers.");
        }
        pwm_ports.insert(std::make_pair(
            "PWMPort0", PWMOutputPort("PWMPort0", {"0", "1", "2", "3"}, {0, 1, 2, 3})));
        pwm_ports.insert(std::make_pair(
            "PWMPort1", PWMOutputPort("PWMPort1", {"0", "1", "2", "3"}, {4, 5, 6, 7})));
        pwm_ports.insert(std::make_pair(
            "PWMPort2", PWMOutputPort("PWMPort2", {"0", "1", "2", "3"}, {8, 9, 10, 11})));
        pwm_ports.insert(std::make_pair(
            "PWMPort3", PWMOutputPort("PWMPort3", {"0", "1", "2", "3"}, {12, 13, 14, 15})));
        int status = driver.init();
        if (status < 0) {
            diagnostic = diag_helper.update_diagnostic(
                Diagnostic::DiagnosticType::COMMUNICATIONS,
                Level::Type::ERROR,
                Diagnostic::Message::INITIALIZING_ERROR,
                "Unable to Start Servo Hat at Address: " + std::to_string(driver.get_address()));
            logger->log_diagnostic(diagnostic);
            return false;
        }
    }
    diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Initialized Hat.");

    return true;
}
bool PWMHat::init_ros(boost::shared_ptr<ros::NodeHandle> _n, std::string host_name) {
    if (_n == nullptr) {
        logger->log_error("Node Handle has No Memory.");
        return false;
    }
    nodeHandle = _n;
    for (auto port : pwm_ports) {
        std::vector<PWMOutputChannel> _channels = port.second.get_channels();
        for (auto ch : _channels) {
            std::string tempstr = "/" + host_name + "/" + name + "/" + port.second.get_name() +
                                  "/" + ch.get_pin_name();
            ros::Subscriber sub =
                nodeHandle->subscribe<std_msgs::Int64>(tempstr,
                                                       1,
                                                       boost::bind(&PWMHat::PWMOutputCallback,
                                                                   this,
                                                                   _1,
                                                                   port.second.get_name(),
                                                                   ch.get_pin_name()));
            pwmoutput_subs.push_back(sub);
        }
    }

    return true;
}
ChannelDefinition::ChannelErrorType PWMHat::update_pin(std::string port_name,
                                                       std::string pin_name,
                                                       int64_t value) {
    auto port = pwm_ports.find(port_name);
    if (port == pwm_ports.end()) {
        return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    }
    ChannelDefinition::ChannelErrorType error = port->second.update(pin_name, value);
    if (error == ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND) {
        return error;
    }
    PWMOutputChannel ch = port->second.get_channel(pin_name);
    driver.setServoValue(ch.get_pin_number(), (int)value);
    return error;
}
void PWMHat::PWMOutputCallback(const std_msgs::Int64::ConstPtr &msg,
                               const std::string &port_name,
                               const std::string &pin_name) {
    int64_t v = msg->data;
    ChannelDefinition::ChannelErrorType error = update_pin(port_name, pin_name, v);
    switch (error) {
        case ChannelDefinition::ChannelErrorType::NOERROR:
            diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                       Level::Type::INFO,
                                                       Diagnostic::Message::NOERROR,
                                                       "Updated");
            return;
        case ChannelDefinition::ChannelErrorType::VALUE_EXCEED_LOWER_BOUND:
            diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                       Level::Type::WARN,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       "Set value exceeded lower bounds");
            logger->log_diagnostic(diagnostic);
            return;
        case ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND:
            diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                       Level::Type::WARN,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       "Set value exceeded higher bounds");
            logger->log_diagnostic(diagnostic);
            return;
        case ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND:
            diagnostic = diag_helper.update_diagnostic(
                Diagnostic::DiagnosticType::ACTUATORS,
                Level::Type::ERROR,
                Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                "Port: " + port_name + " Pin: " + pin_name + " Not Found.");
            logger->log_diagnostic(diagnostic);
            return;
        default:
            diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                                       "Unknown Error");
            logger->log_diagnostic(diagnostic);
            return;
    }
}