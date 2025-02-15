#include <ros_hats/HatDefinition.h>
namespace ros_hats {
std::string HatDefinition::HatTypeString(HatDefinition::HatType v) {
    switch (v) {
        case HatDefinition::HatType::UNKNOWN: return "UNKNOWN"; break;
        case HatDefinition::HatType::SERVO_HAT: return "SERVO_HAT"; break;
        case HatDefinition::HatType::RELAY_HAT: return "RELAY_HAT"; break;
        case HatDefinition::HatType::ARDUINO_HAT: return "ARDUINO_HAT"; break;
        case HatDefinition::HatType::GPS_HAT: return "GPS_HAT"; break;
        default: return HatTypeString(HatDefinition::HatType::UNKNOWN); break;
    }
}
HatDefinition::HatType HatDefinition::HatTypeEnum(std::string v) {
    if (v == "SERVO_HAT") {
        return HatDefinition::HatType::SERVO_HAT;
    }
    else if (v == "RELAY_HAT") {
        return HatDefinition::HatType::RELAY_HAT;
    }
    else if (v == "ARDUINO_HAT") {
        return HatDefinition::HatType::ARDUINO_HAT;
    }
    else if (v == "GPS_HAT") {
        return HatDefinition::HatType::GPS_HAT;
    }
    else if (v == "SERVO_HAT") {
        return HatDefinition::HatType::SERVO_HAT;
    }
    else {
        return HatDefinition::HatType::UNKNOWN;
    }
}
}  // namespace ros_hats