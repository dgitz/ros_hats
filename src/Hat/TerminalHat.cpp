#include <ros_hats/Hat/TerminalHat.h>
std::vector<ros::Publisher>
    ROSHATS_TERMINALHAT_H_digitalinput_pubs;  // Avoid potential global variable conflicts
DigitalInputPort ROSHATS_TERMINALHAT_H_digitalinput_port;
std::vector<std::string> ROSHATS_TERMINALHAT_H_digitalinput_channel_names;
TerminalHat::~TerminalHat() {
}
std::string TerminalHat::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += "Type: TerminalHat Model: " + HatModelString(model) + " --- \n";
    str += ROSHATS_TERMINALHAT_H_digitalinput_port.pretty(pre + "\t") + "\n";
    str += digitaloutput_port.pretty(pre + "\t") + "\n";
    return str;
}
bool TerminalHat::init(Logger *_logger,
                       RaspberryPiDefinition::RaspberryPiModel _board,
                       HatConfig _config) {
    bool v = base_init(_logger, _board);
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
    diag_helper.enable_diagnostics(std::vector<Diagnostic::DiagnosticType>{diagnostic.type});
    diagnostic = diag_helper.update_diagnostic(diagnostic);
    if (model == TerminalHat::HatModel::STANDARD) {
        if (hat_config.use_default_config == true) {
            logger->log_error("Default Config for Terminal Hat Not Supported.  Exiting.");
            return false;
        }
    }
    if (hat_config.ports.size() == 0) {
        logger->log_error("NO Ports Defined. Exiting.");
        return false;
    }
    wiringPiSetupGpio();
    uint16_t digitalinput_port_count = 0;
    uint16_t digitaloutput_port_count = 0;
    for (std::size_t i = 0; i < hat_config.ports.size(); ++i) {
        if (hat_config.ports.at(i).port_type == ChannelDefinition::ChannelType::DIGITAL) {
            if (hat_config.ports.at(i).direction == ChannelDefinition::Direction::CH_INPUT) {
                digitalinput_port_count++;
                ROSHATS_TERMINALHAT_H_digitalinput_port = DigitalInputPort(hat_config.ports.at(i));
            }
            if (hat_config.ports.at(i).direction == ChannelDefinition::Direction::CH_OUTPUT) {
                digitaloutput_port_count++;
                digitaloutput_port = DigitalOutputPort(hat_config.ports.at(i));
            }
        }
    }
    if (digitalinput_port_count > 1) {
        logger->log_error("Only 1 Digital Input Port allowed. Exiting.");
        return false;
    }
    if (digitaloutput_port_count > 1) {
        logger->log_error("Only 1 Digital Output Port allowed. Exiting.");
        return false;
    }

    diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Initialized Hat.");

    return true;
}
bool TerminalHat::update(double dt) {
    return true;
}
bool TerminalHat::init_ros(boost::shared_ptr<ros::NodeHandle> _n, std::string host_name) {
    if (_n == nullptr) {
        logger->log_error("Node Handle has No Memory.");
        return false;
    }
    nodeHandle = _n;
    {  // Setup Digital Inputs
        std::vector<DigitalInputChannel> _channels =
            ROSHATS_TERMINALHAT_H_digitalinput_port.get_channels();
        for (auto ch : _channels) {
            auto found = pi_model.pin_map.right.find(ch.get_pin_number());
            if (found == pi_model.pin_map.right.end()) {
                return false;
            }
            std::string pin_name = found->second;
            uint16_t gpio_pin_number = std::atoi(pin_name.c_str());

            if (ROSHATS_TERMINALHAT_H_digitalinput_pubs.size() == MAXDIGITALINPUT_CALLBACKS) {
                // Don't add more.
                logger->log_warn("Max number of Digital Input callbacks reached. Exiting.");
                return false;
            }
            std::string tempstr = "/" + host_name + "/" + name + "/" +
                                  ROSHATS_TERMINALHAT_H_digitalinput_port.get_name() + "/" +
                                  ch.get_channel_name();
            ROSHATS_TERMINALHAT_H_digitalinput_channel_names.push_back(ch.get_channel_name());
            ros::Publisher pub = nodeHandle->advertise<std_msgs::Bool>(tempstr, 20);
            ROSHATS_TERMINALHAT_H_digitalinput_pubs.push_back(pub);
            switch (ROSHATS_TERMINALHAT_H_digitalinput_pubs.size()) {
                case 1: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_1); break;
                case 2: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_2); break;
                case 3: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_3); break;
                case 4: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_4); break;
                case 5: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_5); break;
                case 6: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_6); break;
                case 7: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_7); break;
                case 8: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_8); break;
                case 9: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_9); break;
                case 10: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_10); break;
                case 11: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_11); break;
                case 12: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_12); break;
                case 13: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_13); break;
                case 14: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_14); break;
                case 15: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_15); break;
                case 16: wiringPiISR(gpio_pin_number, INT_EDGE_RISING, digitalInputCB_16); break;
                default: break;
            }
        }
    }
    {  // Setup Digital Inputs
        std::vector<DigitalOutputChannel> _channels = digitaloutput_port.get_channels();
        for (auto ch : _channels) {
            std::string tempstr = "/" + host_name + "/" + name + "/" +
                                  digitaloutput_port.get_name() + "/" + ch.get_channel_name();
            auto found = pi_model.pin_map.right.find(ch.get_pin_number());
            if (found == pi_model.pin_map.right.end()) {
                return false;
            }
            std::string pin_name = found->second;
            uint16_t gpio_pin_number = std::atoi(pin_name.c_str());
            pinMode(gpio_pin_number, OUTPUT);
            ros::Subscriber sub = nodeHandle->subscribe<std_msgs::Bool>(
                tempstr,
                1,
                boost::bind(&TerminalHat::DigitalOutputCallback, this, _1, ch.get_channel_name()));
            digitaloutput_subs.push_back(sub);
        }
    }
    return true;
}
void TerminalHat::DigitalOutputCallback(const std_msgs::Bool::ConstPtr &msg,
                                        const std::string &channel_name) {
    bool v = msg->data;
    ChannelDefinition::ChannelErrorType error = update_pin(channel_name, (int64_t)v);
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
            diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                                       "Pin: " + channel_name + " Not Found.");
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
ChannelDefinition::ChannelErrorType TerminalHat::update_pin(std::string channel_name,
                                                            int64_t value) {
    ChannelDefinition::ChannelErrorType error = digitaloutput_port.update(channel_name, value);

    if (error != ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND) {
        DigitalOutputChannel ch = digitaloutput_port.get_channel(channel_name);
        auto found = pi_model.pin_map.right.find(ch.get_pin_number());
        if (found == pi_model.pin_map.right.end()) {
            return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
        }
        std::string pin_name = found->second;
        uint16_t gpio_pin_number = std::atoi(pin_name.c_str());
        digitalWrite(gpio_pin_number, value);
    }
    return error;
}
bool TerminalHat::cleanup() {
    bool cleanup_ok = true;
    bool v = true;
    {
        std::vector<DigitalInputChannel> _channels =
            ROSHATS_TERMINALHAT_H_digitalinput_port.get_channels();
        for (auto ch : _channels) {
            auto found = pi_model.pin_map.right.find(ch.get_pin_number());
            if (found == pi_model.pin_map.right.end()) {
                cleanup_ok = false;
                continue;
            }
            std::string pin_name = found->second;
            uint16_t gpio_pin_number = std::atoi(pin_name.c_str());
            pinMode(gpio_pin_number, INPUT);
        }
    }
    {
        std::vector<DigitalOutputChannel> _channels = digitaloutput_port.get_channels();
        for (auto ch : _channels) {
            auto found = pi_model.pin_map.right.find(ch.get_pin_number());
            if (found == pi_model.pin_map.right.end()) {
                cleanup_ok = false;
                continue;
            }
            std::string pin_name = found->second;
            uint16_t gpio_pin_number = std::atoi(pin_name.c_str());
            pinMode(gpio_pin_number, INPUT);
        }
    }
    if (cleanup_ok == true) {
        logger->log_notice("TerminalHat Cleaned Up Successfully.");
    }
    else {
        logger->log_warn("TerminalHat Cleanup Failed.");
    }
    return cleanup_ok;
}
void TerminalHat::update_digitalinput_frompin(uint16_t index) {
    std_msgs::Bool data;
    data.data = true;
    ROSHATS_TERMINALHAT_H_digitalinput_pubs.at(index).publish(data);
    ROSHATS_TERMINALHAT_H_digitalinput_port.update(
        ROSHATS_TERMINALHAT_H_digitalinput_channel_names.at(index), 1);
}
void TerminalHat::digitalInputCB_1(void) {
    update_digitalinput_frompin(0);
}
void TerminalHat::digitalInputCB_2(void) {
    update_digitalinput_frompin(1);
}
void TerminalHat::digitalInputCB_3(void) {
    update_digitalinput_frompin(2);
}
void TerminalHat::digitalInputCB_4(void) {
    update_digitalinput_frompin(3);
}
void TerminalHat::digitalInputCB_5(void) {
    update_digitalinput_frompin(4);
}
void TerminalHat::digitalInputCB_6(void) {
    update_digitalinput_frompin(5);
}
void TerminalHat::digitalInputCB_7(void) {
    update_digitalinput_frompin(6);
}
void TerminalHat::digitalInputCB_8(void) {
    update_digitalinput_frompin(7);
}
void TerminalHat::digitalInputCB_9(void) {
    update_digitalinput_frompin(8);
}
void TerminalHat::digitalInputCB_10(void) {
    update_digitalinput_frompin(9);
}
void TerminalHat::digitalInputCB_11(void) {
    update_digitalinput_frompin(10);
}
void TerminalHat::digitalInputCB_12(void) {
    update_digitalinput_frompin(11);
}
void TerminalHat::digitalInputCB_13(void) {
    update_digitalinput_frompin(12);
}
void TerminalHat::digitalInputCB_14(void) {
    update_digitalinput_frompin(13);
}
void TerminalHat::digitalInputCB_15(void) {
    update_digitalinput_frompin(14);
}
void TerminalHat::digitalInputCB_16(void) {
    update_digitalinput_frompin(15);
}