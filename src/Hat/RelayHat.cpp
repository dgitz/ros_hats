#include <ros_hats/Hat/RelayHat.h>
RelayHat::~RelayHat() {
}
std::string RelayHat::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += "Type: RelayHat Model: " + HatModelString(model) + " --- \n";
    str += relay_port.pretty(pre + "\t") + "\n";
    return str;
}
bool RelayHat::init(Logger *_logger,
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
    if (model == RelayHat::HatModel::RPI_RELAY_HAT) {
        if (hat_config.use_default_config == true) {
            logger->log_warn("[RelayHat] Using Default Values for Model: " +
                             RelayHat::HatModelString(model));
            hat_config.ports = create_default_port_configs();
        }
        else {
            logger->log_error("Only default config currently supported. Exiting.");
            return false;
        }
    }
    relay_port = DigitalOutputPort(hat_config.ports.at(0));
    std::vector<DigitalOutputChannel> _channels = relay_port.get_channels();
    for (auto ch : _channels) {
        int counter = 0;
        int tries = 10;
        bool success = false;
        while (counter < tries) {
            if (export_gpio(ch.get_channel_name()) == true) {
                success = true;
                break;
            }
        }
        if (success == false) {
            logger->log_error("Not able to Export GPIO.  Exiting.");
            return false;
        }
        counter = 0;
        success = false;
        while (counter < tries) {
            if (setdir_gpio(ch.get_channel_name(), "out") == true) {
                success = true;
                break;
            }
        }
        if (success == false) {
            logger->log_error("Not able to Set Direction on GPIO.  Exiting.");
            return false;
        }
        counter = 0;
        success = false;
        while (counter < tries) {
            if (setvalue_gpio(ch.get_channel_name(), std::to_string(ch.get_default_value())) ==
                true) {
                success = true;
                break;
            }
        }
        if (success == false) {
            logger->log_error("Not able to Set Value on GPIO.  Exiting.");
            return false;
        }
    }
    diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Initialized Hat.");

    return true;
}
std::vector<PortConfig> RelayHat::create_default_port_configs() {
    std::vector<PortConfig> default_ports;
    if (model == RelayHat::HatModel::RPI_RELAY_HAT) {
        {
            PortConfig port("DigitalPort0",
                            ChannelDefinition::ChannelType::DIGITAL,
                            ChannelDefinition::Direction::CH_OUTPUT);
            {
                ChannelConfig channel("20",
                                      ChannelDefinition::ChannelType::DIGITAL,
                                      ChannelDefinition::Direction::CH_OUTPUT,
                                      0);
                channel.data_config =
                    std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 1));
                port.channels.push_back(channel);
            }
            {
                ChannelConfig channel("21",
                                      ChannelDefinition::ChannelType::DIGITAL,
                                      ChannelDefinition::Direction::CH_OUTPUT,
                                      1);
                channel.data_config =
                    std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 1));
                port.channels.push_back(channel);
            }
            {
                ChannelConfig channel("26",
                                      ChannelDefinition::ChannelType::DIGITAL,
                                      ChannelDefinition::Direction::CH_OUTPUT,
                                      2);
                channel.data_config =
                    std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 1));
                port.channels.push_back(channel);
            }
            default_ports.push_back(port);
        }
    }
    return default_ports;
}
bool RelayHat::update(double dt) {
    return true;
}
bool RelayHat::init_ros(boost::shared_ptr<ros::NodeHandle> _n, std::string host_name) {
    if (_n == nullptr) {
        logger->log_error("Node Handle has No Memory.");
        return false;
    }
    nodeHandle = _n;
    std::vector<DigitalOutputChannel> _channels = relay_port.get_channels();
    for (auto ch : _channels) {
        std::string tempstr = "/" + host_name + "/" + name + "/" + relay_port.get_name() + "/" +
                              ch.get_channel_name();
        ros::Subscriber sub = nodeHandle->subscribe<std_msgs::Bool>(
            tempstr,
            1,
            boost::bind(&RelayHat::DigitalOutputCallback, this, _1, ch.get_channel_name()));
        relayoutput_subs.push_back(sub);
    }
    return true;
}
ChannelDefinition::ChannelErrorType RelayHat::update_pin(std::string channel_name, int64_t value) {
    ChannelDefinition::ChannelErrorType error = relay_port.update(channel_name, value);

    if (error != ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND) {
        DigitalOutputChannel ch = relay_port.get_channel(channel_name);
        bool v = setvalue_gpio(channel_name, std::to_string(ch.get_value()));
        if (v == false) {
            return ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND;
        }
    }
    return error;
}
void RelayHat::DigitalOutputCallback(const std_msgs::Bool::ConstPtr &msg,
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
bool RelayHat::export_gpio(std::string pin_name) {
    std::string export_str = "/sys/class/gpio/export";
    std::ofstream export_gpio_fd(export_str.c_str());
    if (export_gpio_fd.is_open() == false) {
        std::string tempstr = "OPERATION FAILED: Unable to export GPIO:" + export_str;
        diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::INITIALIZING_ERROR,
                                                   tempstr);
        logger->log_diagnostic(diagnostic);
        return false;
    }

    export_gpio_fd << pin_name;
    export_gpio_fd.close();
    usleep(DELAY_GPIOEXPORT_MS);
    return true;
}
bool RelayHat::unexport_gpio(std::string pin_name) {
    std::string unexport_str = "/sys/class/gpio/unexport";
    std::ofstream unexport_gpio_fd(unexport_str.c_str());
    if (unexport_gpio_fd.is_open() == false) {
        std::string tempstr = "OPERATION FAILED: Unable to unexport GPIO:" + unexport_str;
        diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::INITIALIZING_ERROR,
                                                   tempstr);
        logger->log_diagnostic(diagnostic);
        return false;
    }

    unexport_gpio_fd << pin_name;
    unexport_gpio_fd.close();
    usleep(DELAY_GPIOEXPORT_MS);
    return true;
}
bool RelayHat::setdir_gpio(std::string pin_name, std::string dir) {
    std::string out_str = "/sys/class/gpio/gpio" + pin_name + "/direction";
    std::ofstream setdir_gpio_fd(out_str.c_str());
    if (setdir_gpio_fd.is_open() == false) {
        std::string tempstr = "OPERATION FAILED: Unable to Set Direction on GPIO:" + out_str;
        diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::INITIALIZING_ERROR,
                                                   tempstr);
        logger->log_diagnostic(diagnostic);
        return false;
    }
    setdir_gpio_fd << dir;
    setdir_gpio_fd.close();
    return true;
}
bool RelayHat::setvalue_gpio(std::string pin_name, std::string value) {
    std::string out_str = "/sys/class/gpio/gpio" + pin_name + "/value";
    std::ofstream setvalue_gpio_fd(out_str.c_str());
    if (setvalue_gpio_fd.is_open() == false) {
        std::string tempstr = "OPERATION FAILED: Unable to Set Value on GPIO:" + out_str;
        diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::ACTUATORS,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::INITIALIZING_ERROR,
                                                   tempstr);
        logger->log_diagnostic(diagnostic);
        return false;
    }
    setvalue_gpio_fd << value;
    setvalue_gpio_fd.close();
    return true;
}
bool RelayHat::cleanup() {
    bool cleanup_ok = true;
    std::vector<DigitalOutputChannel> _channels = relay_port.get_channels();
    if (_channels.size() == 0) {
        logger->log_warn("No RelayHat Channels Defined.  Not cleaning up.");
    }
    for (auto ch : _channels) {
        if (setdir_gpio(ch.get_channel_name(), "in") == false) {
            cleanup_ok = false;
            logger->log_warn("Reset Direction Channel: " + ch.get_channel_name() + " Failed.");
        }
        if (unexport_gpio(ch.get_channel_name()) == false) {
            cleanup_ok = false;
            logger->log_warn("Unexport on Channel: " + ch.get_channel_name() + " Failed.");
        }
    }
    if (cleanup_ok == true) {
        logger->log_notice("RelayHat Cleaned Up Successfully.");
    }
    return cleanup_ok;
}