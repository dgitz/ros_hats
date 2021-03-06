#include <ros_hats/nodes/HatNode/HatNodeProcess.h>
using namespace eros;
HatNodeProcess::HatNodeProcess() {
}
HatNodeProcess::~HatNodeProcess() {
}
Diagnostic::DiagnosticDefinition HatNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void HatNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition HatNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> HatNodeProcess::new_commandmsg(eros::command msg) {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> HatNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::string HatNodeProcess::pretty(std::map<std::string, HatConfig> hat_configs) {
    std::string str = "";
    for (auto hat_it : hat_configs) {
        str += "Hat: " + hat_it.second.hat_name + " Type: " + hat_it.second.hat_type +
               " Model: " + hat_it.second.hat_model + "\n";
        for (auto port_it : hat_it.second.ports) {
            str += "\tPort: " + port_it.port_name +
                   " Type: " + ChannelDefinition::ChannelTypeString(port_it.port_type) + "\n";
            for (auto channel_it : port_it.channels) {
                str += "\t\tChannel: " + channel_it.channel_name +
                       " Type: " + ChannelDefinition::ChannelTypeString(channel_it.channel_type) +
                       " Pin Number: " + std::to_string(channel_it.pin_number) + "\n";
            }
        }
    }
    return str;
}
std::map<std::string, HatConfig> HatNodeProcess::load_hat_config(std::string file_path) {
    std::map<std::string, HatConfig> hat_configs;
    file_path = sanitize_path(file_path);
    json json_obj = read_configuration(hostname, false, file_path);
    std::string device_type, device_model;
    uint16_t hat_counter = 0;
    uint16_t port_counter = 0;
    uint16_t channel_counter = 0;
    for (auto& [name, obj] : json_obj.items()) {
        logger->log_debug("Loading: " + name);
        auto find_type = obj.find("Type");
        if (find_type != obj.end()) {
            device_type = *find_type;
            if ((device_type == "ServoHat") || (device_type == "RelayHat") ||
                (device_type == "GPSHat") || (device_type == "TerminalHat") ||
                (device_type == "DisplayHat")) {
                auto find_model = obj.find("Model");
                if (find_model != obj.end()) {
                    device_model = *find_model;
                }
                HatConfig hat_config;
                hat_config.hat_name = name;
                hat_config.hat_type = device_type;
                hat_config.hat_model = device_model;
                auto use_default_config = obj.find("UseDefaultConfig");
                if (use_default_config == obj.end()) {
                    hat_config.use_default_config = true;
                }
                else {
                    hat_config.use_default_config = *use_default_config;
                }
                port_counter = 0;
                channel_counter = 0;

                for (auto& port_it : obj["Ports"].items()) {
                    PortConfig port;
                    auto port_name = port_it.value().find("Name");
                    port.port_name = *port_name;
                    auto port_type = port_it.value().find("Type");
                    auto port_direction = port_it.value().find("Direction");
                    std::string tempstr = *port_type;
                    port.port_type = ChannelDefinition::ChannelTypeEnum(*port_type);
                    port.direction = ChannelDefinition::DirectionEnum(*port_direction);
                    channel_counter = 0;
                    for (auto& channel_it : port_it.value()["Channels"].items()) {
                        ChannelConfig channel;
                        auto channel_name = channel_it.value().find("ChannelName");
                        channel.channel_name = *channel_name;
                        auto channel_type = channel_it.value().find("Type");
                        channel.channel_type = ChannelDefinition::ChannelTypeEnum(*channel_type);
                        if (channel.channel_type == ChannelDefinition::ChannelType::DIGITAL) {
                            auto pin_number = channel_it.value().find("PinNumber");
                            channel.pin_number = *pin_number;
                            auto direction = channel_it.value().find("Direction");
                            channel.direction = ChannelDefinition::DirectionEnum(*direction);
                            auto default_value = channel_it.value().find("DefaultValue");
                            auto max_value = channel_it.value().find("MaxValue");
                            auto min_value = channel_it.value().find("MinValue");
                            int64_t v1 = *default_value;
                            int64_t v2 = *min_value;
                            int64_t v3 = *max_value;
                            channel.data_config = std::make_shared<DigitalChannelDataConfig>(
                                DigitalChannelDataConfig(v1, v2, v3));
                        }
                        else if (channel.channel_type == ChannelDefinition::ChannelType::SERVO) {
                            auto pin_number = channel_it.value().find("PinNumber");
                            channel.pin_number = *pin_number;
                            auto direction = channel_it.value().find("Direction");
                            channel.direction = ChannelDefinition::DirectionEnum(*direction);
                            auto default_value = channel_it.value().find("DefaultValue");
                            auto max_value = channel_it.value().find("MaxValue");
                            auto min_value = channel_it.value().find("MinValue");
                            int64_t v1 = *default_value;
                            int64_t v2 = *min_value;
                            int64_t v3 = *max_value;
                            channel.data_config = std::make_shared<ServoChannelDataConfig>(
                                ServoChannelDataConfig(v1, v2, v3));
                        }
                        channel_counter++;
                        port.channels.push_back(channel);
                    }
                    if (channel_counter == 0) {
                        logger->log_warn("No Channels Defined for this Port.  Not adding.");
                    }
                    else {
                        port_counter++;
                        hat_config.ports.push_back(port);
                    }
                }
                if (port_counter == 0) {
                    logger->log_warn("No Ports Defined for this Hat.  Not adding.");
                }
                else {
                    hat_configs.insert(std::make_pair(hat_config.hat_name, hat_config));
                    hat_counter++;
                }
            }
        }
    }
    if (hat_counter == 0) {
        logger->log_warn("No Hats Defined.");
    }
    return hat_configs;
}