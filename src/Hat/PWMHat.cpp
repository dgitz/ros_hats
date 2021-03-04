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
bool PWMHat::init(Logger *_logger, HatConfig _config) {
    bool v = base_init(_logger);
    if (v == false) {
        return false;
    }

    if ((model == HatModel::UNKNOWN) || (model == HatModel::END_OF_LIST)) {
        return false;
    }
    hat_config = _config;
    diagnostic.device_name = hat_config.hat_name;
    diagnostic.node_name = hat_config.hat_name;
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
    if (model == PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH) {
        if (hat_config.use_default_config == true) {
            hat_config.ports = create_default_port_configs();
            logger->log_warn("Using Default Values for Model: " + PWMHat::HatModelString(model));
        }
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
    for (auto port : hat_config.ports) {
        pwm_ports.insert(std::make_pair(port.port_name, PWMOutputPort(port)));
    }
    // relay_port = DigitalOutputPort(hat_config.ports.at(0), {0, 0, 0}, {0, 0, 0}, {1, 1, 1});
    diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Initialized Hat.");

    return true;
}
std::vector<PortConfig> PWMHat::create_default_port_configs() {
    std::vector<PortConfig> default_ports;
    if (model == PWMHat::HatModel::ADAFRUIT_SERVOHAT_16CH) {
        {
            uint16_t counter = 0;
            for (int i = 0; i < 4; ++i) {
                PortConfig port("PWMPort" + std::to_string(i), ChannelDefinition::ChannelType::PWM);
                for (int j = 0; j < 4; ++j) {
                    ChannelConfig channel(std::to_string(j),
                                          ChannelDefinition::ChannelType::PWM,
                                          ChannelDefinition::Direction::OUTPUT,
                                          counter);
                    channel.data_config = std::make_shared<PWMChannelDataConfig>(
                        PWMChannelDataConfig(1500, 1000, 2000));
                    port.channels.push_back(channel);
                    counter++;
                }
                default_ports.push_back(port);
            }
        }
    }
    return default_ports;
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
                                  "/" + ch.get_channel_name();
            ros::Subscriber sub =
                nodeHandle->subscribe<std_msgs::Int64>(tempstr,
                                                       1,
                                                       boost::bind(&PWMHat::PWMOutputCallback,
                                                                   this,
                                                                   _1,
                                                                   port.second.get_name(),
                                                                   ch.get_channel_name()));
            pwmoutput_subs.push_back(sub);
        }
    }

    return true;
}
ChannelDefinition::ChannelErrorType PWMHat::update_pin(std::string port_name,
                                                       std::string channel_name,
                                                       int64_t value) {
    auto port = pwm_ports.find(port_name);
    if (port == pwm_ports.end()) {
        return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    }
    ChannelDefinition::ChannelErrorType error = port->second.update(channel_name, value);
    if (error == ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND) {
        return error;
    }
    PWMOutputChannel ch = port->second.get_channel(channel_name);
    driver.setServoValue(ch.get_pin_number(), (int)value);
    return error;
}
void PWMHat::PWMOutputCallback(const std_msgs::Int64::ConstPtr &msg,
                               const std::string &port_name,
                               const std::string &channel_name) {
    int64_t v = msg->data;
    ChannelDefinition::ChannelErrorType error = update_pin(port_name, channel_name, v);
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
                "Port: " + port_name + " Channel: " + channel_name + " Not Found.");
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