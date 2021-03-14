#include <ros_hats/Hat/GPSHat.h>
GPSHat::~GPSHat() {
}
std::string GPSHat::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += "Type: GPSHat Model: " + HatModelString(model) + " --- \n";
    str += gps_port.pretty(pre + "\t") + "\n";
    return str;
}
bool GPSHat::init(Logger *_logger,
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
    diagnostic.component = System::Component::POSE;
    diagnostic.type = Diagnostic::DiagnosticType::SENSORS;
    diagnostic.message = Diagnostic::Message::INITIALIZING;
    diagnostic.level = Level::Type::INFO;
    diagnostic.description = "Hat Initializing";
    diag_helper.initialize(diagnostic);
    diag_helper.enable_diagnostics(std::vector<Diagnostic::DiagnosticType>{diagnostic.type});
    diagnostic = diag_helper.update_diagnostic(diagnostic);
    if (model == GPSHat::HatModel::STANDARD) {
        if (hat_config.use_default_config == true) {
            logger->log_warn("[GPSHat] Using Default Values for Model: " +
                             GPSHat::HatModelString(model));
            hat_config.ports = create_default_port_configs();
        }
        else {
            logger->log_error("Only default config currently supported. Exiting.");
            return false;
        }
    }
    gps_port = GPSInputPort(hat_config.ports.at(0));
    driver = new GPSHatDriver();
    v = driver->init(logger);
    if (v == false) {
        diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::SENSORS,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::INITIALIZING_ERROR,
                                                   "Unable to initialize GPS Driver.  Exiting.");
        logger->log_diagnostic(diagnostic);
        return false;
    }
    diagnostic = diag_helper.update_diagnostic(Diagnostic::DiagnosticType::SENSORS,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Initialized Hat.");

    return true;
}
std::vector<PortConfig> GPSHat::create_default_port_configs() {
    std::vector<PortConfig> default_ports;
    if (model == GPSHat::HatModel::STANDARD) {
        {
            PortConfig port("GPSPort0",
                            ChannelDefinition::ChannelType::GPS,
                            ChannelDefinition::Direction::CH_INPUT);
            {
                ChannelConfig channel("GPS0",
                                      ChannelDefinition::ChannelType::GPS,
                                      ChannelDefinition::Direction::CH_INPUT,
                                      0);
                channel.data_config =
                    std::make_shared<GPSChannelDataConfig>(GPSChannelDataConfig());
                port.channels.push_back(channel);
            }
            default_ports.push_back(port);
        }
    }
    return default_ports;
}
bool GPSHat::init_ros(boost::shared_ptr<ros::NodeHandle> _n, std::string host_name) {
    if (_n == nullptr) {
        logger->log_error("Node Handle has No Memory.");
        return false;
    }
    nodeHandle = _n;
    std::vector<GPSInputChannel> _channels = gps_port.get_channels();
    for (auto ch : _channels) {
        std::string tempstr =
            "/" + host_name + "/" + name + "/" + gps_port.get_name() + "/" + ch.get_channel_name();
        ros::Publisher pub = nodeHandle->advertise<sensor_msgs::NavSatFix>(tempstr, 1);

        gps_navsat_pubs.push_back(pub);
    }
    ros_initialized = true;
    return true;
}
bool GPSHat::update(double dt) {
    bool v = driver->update(dt);
    if (v == true) {
        gps_port.update("GPS0", driver->get_position(), driver->get_status());
        std::vector<GPSInputChannel> _channels = gps_port.get_channels();
        if (ros_initialized == true) {
            uint16_t i = 0;
            for (auto ch : _channels) {
                sensor_msgs::NavSatFix gps_data = convert(ch);
                gps_navsat_pubs.at(i).publish(gps_data);
                i++;
            }
        }
    }

    return v;
}
sensor_msgs::NavSatFix GPSHat::convert(GPSInputChannel ch) {
    sensor_msgs::NavSatFix msg;
    GPSInputChannel::Position position = ch.get_position();
    GPSInputChannel::Status status = ch.get_status();
    msg.latitude = position.latitude;
    msg.longitude = position.longitude;
    msg.altitude = position.altitude;
    switch (status.fix_type) {
        case GPSInputChannel::FixType::UNKNOWN:
            msg.status.status = -1;  // STATUS_NO_FIX, collides with gps.h STATUS_NO_FIX
            break;
        case GPSInputChannel::FixType::NO_FIX:
            msg.status.status = -1;  // STATUS_NO_FIX, collides with gps.h STATUS_NO_FIX
            break;
        case GPSInputChannel::FixType::FIX_GPS:
            msg.status.status = 0;  // STATUS_FIX, collides with gps.h STATUS_FIX
            break;
        case GPSInputChannel::FixType::FIX_DGPS:
            msg.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
            break;
        default: break;
    }
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time(position.timestamp);

    return msg;
}
bool GPSHat::cleanup() {
    bool cleanup_ok = true;
    bool v = driver->finish();
    if (v == false) {
        cleanup_ok = false;
    }
    if (driver != nullptr) {
        delete driver;
    }
    driver = nullptr;
    if (cleanup_ok == true) {
        logger->log_notice("GPSHat Cleaned Up Successfully.");
    }
    return cleanup_ok;
}