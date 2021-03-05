#include <ros_hats/Hat/TerminalHat.h>
TerminalHat::~TerminalHat() {
}
std::string TerminalHat::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += "Type: TerminalHat Model: " + HatModelString(model) + " --- \n";

    if (digital_output_ports.size() == 0) {
        str += pre + "\tNO Digital Output Ports Defined.\n";
    }
    else {
        uint16_t counter = 0;
        for (auto port : digital_output_ports) {
            str += pre + "Port: " + std::to_string(counter);
            str += port.second.pretty(pre + "\t") + "\n";
            counter++;
        }
    }
    if (digital_input_ports.size() == 0) {
        str += pre + "\tNO Digital Input Ports Defined.\n";
    }
    else {
        uint16_t counter = 0;
        for (auto port : digital_input_ports) {
            str += pre + "Port: " + std::to_string(counter);
            str += port.second.pretty(pre + "\t") + "\n";
            counter++;
        }
    }
    return str;
}
bool TerminalHat::init(Logger *_logger, HatConfig _config) {
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
    if (model == TerminalHat::HatModel::GENERIC) {
        if (hat_config.use_default_config == true) {
            diagnostic =
                diag_helper.update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                              Level::Type::ERROR,
                                              Diagnostic::Message::INITIALIZING_ERROR,
                                              "No Default Values are supported for Terminal Hat: " +
                                                  TerminalHat::HatModelString(model));
            logger->log_diagnostic(diagnostic);
            return false;
        }
    }
    for (auto port : hat_config.ports) {
        if (port.port_type == ChannelDefinition::ChannelType::DIGITAL) {
            if (port.direction == ChannelDefinition::Direction::OUTPUT) {
                digital_output_ports.insert(
                    std::make_pair(port.port_name, DigitalOutputPort(port)));
            }
            else if (port.direction == ChannelDefinition::Direction::INPUT) {
                digital_input_ports.insert(std::make_pair(port.port_name, DigitalInputPort(port)));
            }
        }
    }
    diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Initialized Hat.");

    return true;
}
bool TerminalHat::init_ros(boost::shared_ptr<ros::NodeHandle> _n, std::string host_name) {
    if (_n == nullptr) {
        logger->log_error("Node Handle has No Memory.");
        return false;
    }
    nodeHandle = _n;
    XXX NEXT ADD SUBSCRIBERS / PUBLISHERS
        /*
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
        */
        return true;
}
ChannelDefinition::ChannelErrorType TerminalHat::update_pin(std::string port_name,
                                                            std::string channel_name,
                                                            int64_t value) {
    ChannelDefinition::ChannelErrorType error =
        ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    /*
    auto port = pwm_ports.find(port_name);
    if (port == pwm_ports.end()) {
        return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
    }
    ChannelDefinition::ChannelErrorType error = port->second.update(channel_name, value);
    if (error == ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND) {
        return error;
    }
    PWMOutputChannel ch = port->second.get_channel(channel_name);
    driver.setServoValue(ch.get_pin_number(), ch.get_value());
    */
    return error;
}
bool TerminalHat::cleanup() {
    logger->log_notice("TerminalHat Cleaned Up.");
    return true;
}