/*! \file ROSHATS_Definitions.h
 */
#ifndef ROSHATS_DEFINITIONS_H
#define ROSHATS_DEFINITIONS_H
#include <eros/eROS_Definitions.h>
#include <stdio.h>

#include <boost/bimap.hpp>
#include <map>
#include <memory>
#include <vector>
/*! \class Hat
    \brief Hat class
    Holds Hat Definitions
*/
class HatDefinition
{
   public:
    enum class HatType {
        UNKNOWN = 0,     /*!< Uninitialized value. */
        SERVO_HAT = 1,   /*!< A Hat that is capable of Servo Outputs.*/
        RELAY_HAT = 2,   /*!< A Hat that is capable of Relay Outputs. */
        ARDUINO_HAT = 3, /*!< A Hat that has an onboard Arduino. */
        GPS_HAT = 4,     /*!< A Hat that has a GPS Receiver. */
        END_OF_LIST = 5  /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert HatDefinition::HatType to human readable string
    /*!
      \param v HatDefinition::HatType type
      \return The converted string.
    */
    static std::string HatTypeString(HatDefinition::HatType v) {
        switch (v) {
            case HatDefinition::HatType::UNKNOWN: return "UNKNOWN"; break;
            case HatDefinition::HatType::SERVO_HAT: return "SERVO_HAT"; break;
            case HatDefinition::HatType::RELAY_HAT: return "RELAY_HAT"; break;
            case HatDefinition::HatType::ARDUINO_HAT: return "ARDUINO_HAT"; break;
            case HatDefinition::HatType::GPS_HAT: return "GPS_HAT"; break;
            default: return HatTypeString(HatDefinition::HatType::UNKNOWN); break;
        }
    }
};

/*! \class ChannelDefinition
    \brief ChannelDefinition class
    Holds ChannelDefinition Definitions
*/
class ChannelDefinition
{
   public:
    enum class ChannelErrorType {
        UNKNOWN = 0,                  /*!< Uninitialized value. */
        NOERROR = 1,                  /*!< Channel has no Error. */
        CHANNEL_NOT_FOUND = 2,        /*!< Channel Lookup Failed. */
        VALUE_EXCEED_LOWER_BOUND = 3, /*!< Channel Value exceeded Lower Bound, Saturated. */
        VALUE_EXCEED_UPPER_BOUND = 4, /*!< Channel Value exceeded Upper Bound, Saturated. */
        END_OF_LIST = 5               /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert ChannelDefinition::ChannelErrorType to human readable string
    /*!
      \param v ChannelDefinition::ChannelErrorType type
      \return The converted string.
    */
    static std::string ChannelErrorString(ChannelDefinition::ChannelErrorType v) {
        switch (v) {
            case ChannelDefinition::ChannelErrorType::UNKNOWN: return "UNKNOWN"; break;
            case ChannelDefinition::ChannelErrorType::NOERROR: return "NOERROR"; break;
            case ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND:
                return "CHANNEL_NOT_FOUND";
                break;
            case ChannelDefinition::ChannelErrorType::VALUE_EXCEED_LOWER_BOUND:
                return "VALUE_EXCEED_LOWER_BOUND";
                break;
            case ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND:
                return "VALUE_EXCEED_UPPER_BOUND";
                break;
            default: return ChannelErrorString(ChannelDefinition::ChannelErrorType::UNKNOWN); break;
        }
    }
    enum class ChannelType {
        UNKNOWN = 0, /*!< Uninitialized value. */
        DIGITAL = 1, /*!< A Digital Channel represnts a value ranging from -long/2 to long/2 */
        ANALOG = 2,  /*!< An Analog Channel represnts a value ranging from -double/2 to double/2 */
        SERVO = 3,   /*!< A Servo Channel is a special type of Digital Channel that only supports
                      values   ranging from 0-4096.*/
        GPS = 4,     /*!< Generic GPS Channel*/
        END_OF_LIST = 5 /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert ChannelDefinition::ChannelType to human readable string
    /*!
      \param v ChannelDefinition::ChannelType type
      \return The converted string.
    */
    static std::string ChannelTypeString(ChannelDefinition::ChannelType v) {
        switch (v) {
            case ChannelDefinition::ChannelType::UNKNOWN: return "UNKNOWN"; break;
            case ChannelDefinition::ChannelType::DIGITAL: return "DIGITAL"; break;
            case ChannelDefinition::ChannelType::ANALOG: return "ANALOG"; break;
            case ChannelDefinition::ChannelType::SERVO: return "SERVO"; break;
            case ChannelDefinition::ChannelType::GPS: return "GPS"; break;
            default: return ChannelTypeString(ChannelDefinition::ChannelType::UNKNOWN); break;
        }
    }
    static ChannelType ChannelTypeEnum(std::string v) {
        if (v == "DIGITAL") {
            return ChannelDefinition::ChannelType::DIGITAL;
        }
        else if (v == "ANALOG") {
            return ChannelDefinition::ChannelType::ANALOG;
        }
        else if (v == "SERVO") {
            return ChannelDefinition::ChannelType::SERVO;
        }
        else if (v == "GPS") {
            return ChannelDefinition::ChannelType::GPS;
        }
        else {
            return ChannelDefinition::ChannelType::UNKNOWN;
        }
    }

    enum class Direction {
        UNKNOWN = 0,   /*!< Uninitialized value. */
        CH_INPUT = 1,  /*!< A Channel that only acts as an Input (relative to the device that is
                       using  this Channel). */
        CH_OUTPUT = 2, /*!< A Channel that only acts as an Output (relative to the device that is
                       using this Channel). */
        CH_INPUTOUTPUT = 3, /*!< A Channel that can act as an input or an output (relative to the
                            device that is using this Channel). */
        END_OF_LIST = 4,    /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert ChannelDefinition::Direction to human readable string
    /*!
      \param v ChannelDefinition::Direction type
      \return The converted string.
    */
    static std::string ChannelDirectionString(ChannelDefinition::Direction v) {
        switch (v) {
            case ChannelDefinition::Direction::UNKNOWN: return "UNKNOWN"; break;
            case ChannelDefinition::Direction::CH_INPUT: return "INPUT"; break;
            case ChannelDefinition::Direction::CH_OUTPUT: return "OUTPUT"; break;
            case ChannelDefinition::Direction::CH_INPUTOUTPUT: return "INPUTOUTPUT"; break;
            default: return ChannelDirectionString(ChannelDefinition::Direction::UNKNOWN); break;
        }
    }

    static Direction DirectionEnum(std::string v) {
        if (v == "INPUT") {
            return ChannelDefinition::Direction::CH_INPUT;
        }
        else if (v == "OUTPUT") {
            return ChannelDefinition::Direction::CH_OUTPUT;
        }
        else if (v == "INPUTOUTPUT") {
            return ChannelDefinition::Direction::CH_INPUTOUTPUT;
        }
        else {
            ChannelDefinition::Direction::UNKNOWN;
        }
        return ChannelDefinition::Direction::UNKNOWN;
    }
};
struct ChannelDataConfig {};
struct DigitalChannelDataConfig : public ChannelDataConfig {
    DigitalChannelDataConfig(int64_t _default_value, int64_t _min_value, int64_t _max_value)
        : default_value(_default_value), min_value(_min_value), max_value(_max_value) {
    }
    int64_t default_value;
    int64_t min_value;
    int64_t max_value;
};
struct ServoChannelDataConfig : public ChannelDataConfig {
    ServoChannelDataConfig(int64_t _default_value, int64_t _min_value, int64_t _max_value)
        : default_value(_default_value), min_value(_min_value), max_value(_max_value) {
    }
    int64_t default_value;
    int64_t min_value;
    int64_t max_value;
};
struct AnalogChannelDataConfig : public ChannelDataConfig {
    double default_value;
    double min_value;
    double max_value;
};
struct GPSChannelDataConfig : public ChannelDataConfig {};
struct ChannelConfig {
    ChannelConfig() {
    }
    ChannelConfig(std::string _channel_name,
                  ChannelDefinition::ChannelType _channel_type,
                  ChannelDefinition::Direction _direction,
                  uint16_t _pin_number)
        : channel_name(_channel_name),
          channel_type(_channel_type),
          direction(_direction),
          pin_number(_pin_number) {
    }
    std::string channel_name;
    ChannelDefinition::ChannelType channel_type;
    ChannelDefinition::Direction direction;
    uint16_t pin_number;
    std::shared_ptr<ChannelDataConfig> data_config;
};
struct PortConfig {
    PortConfig() {
    }
    PortConfig(std::string _port_name,
               ChannelDefinition::ChannelType _port_type,
               ChannelDefinition::Direction _direction)
        : port_name(_port_name), port_type(_port_type), direction(_direction) {
    }
    PortConfig(std::string _port_name,
               ChannelDefinition::ChannelType _port_type,
               ChannelDefinition::Direction _direction,
               std::vector<ChannelConfig> _channels)
        : port_name(_port_name), port_type(_port_type), direction(_direction), channels(_channels) {
    }
    std::string port_name;
    ChannelDefinition::ChannelType port_type;
    ChannelDefinition::Direction direction;
    std::vector<ChannelConfig> channels;
};
struct HatConfig {
    HatConfig() {
    }
    HatConfig(std::string _hat_name,
              std::string _hat_type,
              std::string _hat_model,
              bool _use_default_config)
        : hat_name(_hat_name),
          hat_type(_hat_type),
          hat_model(_hat_model),
          use_default_config(_use_default_config) {
    }
    std::string hat_name;
    std::string hat_type;
    std::string hat_model;
    bool use_default_config;
    std::vector<PortConfig> ports;
};

class RaspberryPiDefinition
{
   public:
    static constexpr const char* boardversion_check =
        "cat /proc/cpuinfo | grep 'Revision' | awk '{print $3}' | sed 's/^1000//'";
    typedef boost::bimap<std::string, uint16_t> pin_map_type;
    enum class RaspberryPiModel {
        UNKNOWN = 0,                   /*!< Uninitialized value. */
        RASPBERRYPI_2_MODEL_B = 1,     /*!< Raspberry Pi 2 ModelB. */
        RASPBERRYPI_3_MODEL_B = 2,     /*!<  */
        RASPBERRYPI_3_MODEL_APLUS = 3, /*!<  */
        RASPBERRYPI_3_MODEL_BPLUS = 4, /*!<  */
        RASPBERRYPI_4_MODEL_B = 5,     /*!<  */
        RASPBERRYPI_ZER0 = 6,          /*!<  */
        RASPBERRYPI_ZERO_W = 7,        /*!<  */
        RASPBERRYPI_ZERO_WH = 8,       /*!<  */
        RASPBERRYPI_400 = 9,           /*!<  */
        END_OF_LIST = 10               /*!< Last item of list. Used for Range Checks. */
    };
    static RaspberryPiDefinition::RaspberryPiModel RaspberryPiModelFromVersion(std::string v) {
        // Source: https://elinux.org/RPi_HardwareHistory
        if (v == "a01041") {
            return RaspberryPiModel::RASPBERRYPI_2_MODEL_B;
        }
        else if (v == "a21041") {
            return RaspberryPiModel::RASPBERRYPI_2_MODEL_B;
        }
        else if (v == "a22042") {
            return RaspberryPiModel::RASPBERRYPI_2_MODEL_B;
        }
        else if (v == "900092") {
            return RaspberryPiModel::RASPBERRYPI_ZER0;
        }
        else if (v == "900093") {
            return RaspberryPiModel::RASPBERRYPI_ZER0;
        }
        else if (v == "920093") {
            return RaspberryPiModel::RASPBERRYPI_ZER0;
        }
        else if (v == "9000c1") {
            return RaspberryPiModel::RASPBERRYPI_ZERO_W;
        }
        else if (v == "a02082") {
            return RaspberryPiModel::RASPBERRYPI_3_MODEL_B;
        }
        else if (v == "a22082") {
            return RaspberryPiModel::RASPBERRYPI_3_MODEL_B;
        }
        else if (v == "a32082") {
            return RaspberryPiModel::RASPBERRYPI_3_MODEL_B;
        }
        else if (v == "a020d3") {
            return RaspberryPiModel::RASPBERRYPI_3_MODEL_BPLUS;
        }
        else if (v == "9020e0") {
            return RaspberryPiModel::RASPBERRYPI_3_MODEL_APLUS;
        }
        else if (v == "a03111") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "b03111") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "b03112") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "b03114") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "c03111") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "c03112") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "c03114") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "d03114") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else if (v == "") {
            return RaspberryPiModel::RASPBERRYPI_4_MODEL_B;
        }
        else {
            return RaspberryPiModel::UNKNOWN;
        }
    }
    static std::string RaspberryPiModelString(RaspberryPiDefinition::RaspberryPiModel v) {
        switch (v) {
            case RaspberryPiDefinition::RaspberryPiModel::UNKNOWN: return "UNKNOWN"; break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_2_MODEL_B:
                return "RASPBERRYPI_2_MODEL_B";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_3_MODEL_B:
                return "RASPBERRYPI_3_MODEL_B";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_3_MODEL_APLUS:
                return "RASPBERRYPI_3_MODEL_APLUS";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_3_MODEL_BPLUS:
                return "RASPBERRYPI_3_MODEL_BPLUS";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_4_MODEL_B:
                return "RASPBERRYPI_4_MODEL_B";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_ZER0:
                return "RASPBERRYPI_ZER0";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_ZERO_W:
                return "RASPBERRYPI_ZERO_W";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_ZERO_WH:
                return "RASPBERRYPI_ZERO_WH";
                break;
            case RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_400:
                return "RASPBERRYPI_400";
                break;
            default:
                return RaspberryPiModelString(RaspberryPiDefinition::RaspberryPiModel::UNKNOWN);
                break;
        }
    }

    enum class PinType {
        UNKNOWN = 0,    /*!< Uninitialized value. */
        GPIO = 1,       /*!< General Purpose Input/Output */
        I2C = 2,        /*!< I2C Pin */
        SPI = 3,        /*!< SPI Pin */
        PWM = 4,        /*!< Pin that supports PWM */
        UART = 5,       /*!< UART Pin */
        END_OF_LIST = 6 /*!< Last item of list. Used for Range Checks. */
    };
    static std::string PinTypeString(RaspberryPiDefinition::PinType v) {
        switch (v) {
            case RaspberryPiDefinition::PinType::UNKNOWN: return "UNKNOWN"; break;
            case RaspberryPiDefinition::PinType::GPIO: return "GPIO"; break;
            case RaspberryPiDefinition::PinType::I2C: return "I2C"; break;
            case RaspberryPiDefinition::PinType::SPI: return "SPI"; break;
            case RaspberryPiDefinition::PinType::PWM: return "PWM"; break;
            case RaspberryPiDefinition::PinType::UART: return "UART"; break;
            default: return PinTypeString(RaspberryPiDefinition::PinType::UNKNOWN); break;
        }
    }

    struct PinDefinition {
        PinDefinition(std::string _gpio_pin_name, std::vector<PinType> _pin_types)
            : gpio_pin_name(_gpio_pin_name), pin_types(_pin_types) {
        }
        std::string gpio_pin_name;
        std::vector<PinType> pin_types;
    };

    struct RaspberryPi {
        RaspberryPi() {
        }
        RaspberryPi(RaspberryPiModel _model, std::map<std::string, PinDefinition> _pin_defines)
            : model(_model), pin_defines(_pin_defines) {
        }
        RaspberryPiModel model;
        std::map<std::string, PinDefinition> pin_defines;
        pin_map_type pin_map;
    };

    static std::string pretty(RaspberryPi device) {
        std::string str = "Raspberry Pi Model: " + RaspberryPiModelString(device.model) + "\n";
        uint16_t counter = 0;
        for (auto pin : device.pin_defines) {
            auto pin_number_it = device.pin_map.left.find(pin.second.gpio_pin_name);
            str += "\t[" + std::to_string(counter) + "] GPIO Name: " + pin.second.gpio_pin_name +
                   " Number: " + std::to_string(pin_number_it->second) + " Type(s):";
            if (pin.second.pin_types.size() == 0) {
                str += " NONE DEFINED.\n";
            }
            else if (pin.second.pin_types.size() == 1) {
                str += " " + PinTypeString(pin.second.pin_types.at(0)) + "\n";
            }
            else {
                str += " " + PinTypeString(pin.second.pin_types.at(0));
                for (std::size_t i = 1; i < pin.second.pin_types.size(); ++i) {
                    str += "," + PinTypeString(pin.second.pin_types.at(i));
                }
                str += "\n";
            }
            counter++;
        }
        return str;
    }

    static std::vector<RaspberryPi> build_RaspberryPi() {
        std::vector<RaspberryPi> devices;
        for (uint8_t i = 1; i < (uint8_t)(RaspberryPiDefinition::RaspberryPiModel::END_OF_LIST);
             ++i) {
            devices.push_back(build_RaspberryPi((RaspberryPiDefinition::RaspberryPiModel)i));
        }
        return devices;
    }
    static RaspberryPi build_RaspberryPi(RaspberryPiModel model) {
        RaspberryPi device;
        device.model = model;
        std::string temp_gpio_pin_name;
        if ((model == RaspberryPiModel::RASPBERRYPI_2_MODEL_B) ||
            (model == RaspberryPiModel::RASPBERRYPI_3_MODEL_APLUS) ||
            (model == RaspberryPiModel::RASPBERRYPI_3_MODEL_B) ||
            (model == RaspberryPiModel::RASPBERRYPI_3_MODEL_BPLUS) ||
            (model == RaspberryPiModel::RASPBERRYPI_400) ||
            (model == RaspberryPiModel::RASPBERRYPI_4_MODEL_B) ||
            (model == RaspberryPiModel::RASPBERRYPI_ZER0) ||
            (model == RaspberryPiModel::RASPBERRYPI_ZERO_W) ||
            (model == RaspberryPiModel::RASPBERRYPI_ZERO_WH)) {
            device.pin_map.insert(pin_map_type::value_type("0", 27));
            device.pin_map.insert(pin_map_type::value_type("1", 28));
            device.pin_map.insert(pin_map_type::value_type("2", 3));
            device.pin_map.insert(pin_map_type::value_type("3", 5));
            device.pin_map.insert(pin_map_type::value_type("4", 7));
            device.pin_map.insert(pin_map_type::value_type("5", 29));
            device.pin_map.insert(pin_map_type::value_type("6", 31));
            device.pin_map.insert(pin_map_type::value_type("7", 26));
            device.pin_map.insert(pin_map_type::value_type("8", 24));
            device.pin_map.insert(pin_map_type::value_type("9", 21));
            device.pin_map.insert(pin_map_type::value_type("10", 19));
            device.pin_map.insert(pin_map_type::value_type("11", 23));
            device.pin_map.insert(pin_map_type::value_type("12", 32));
            device.pin_map.insert(pin_map_type::value_type("13", 33));
            device.pin_map.insert(pin_map_type::value_type("14", 8));
            device.pin_map.insert(pin_map_type::value_type("15", 10));
            device.pin_map.insert(pin_map_type::value_type("16", 36));
            device.pin_map.insert(pin_map_type::value_type("17", 11));
            device.pin_map.insert(pin_map_type::value_type("18", 12));
            device.pin_map.insert(pin_map_type::value_type("19", 35));
            device.pin_map.insert(pin_map_type::value_type("20", 38));
            device.pin_map.insert(pin_map_type::value_type("21", 40));
            device.pin_map.insert(pin_map_type::value_type("22", 15));
            device.pin_map.insert(pin_map_type::value_type("23", 16));
            device.pin_map.insert(pin_map_type::value_type("24", 18));
            device.pin_map.insert(pin_map_type::value_type("25", 22));
            device.pin_map.insert(pin_map_type::value_type("26", 37));
            device.pin_map.insert(pin_map_type::value_type("27", 13));
            temp_gpio_pin_name = "0";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "1";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "2";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::I2C})));
            temp_gpio_pin_name = "3";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::I2C})));
            temp_gpio_pin_name = "4";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "14";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::UART})));
            temp_gpio_pin_name = "15";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::UART})));
            temp_gpio_pin_name = "17";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "18";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "27";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "22";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "23";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "24";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "10";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::SPI})));
            temp_gpio_pin_name = "9";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::SPI})));
            temp_gpio_pin_name = "25";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "11";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::SPI})));
            temp_gpio_pin_name = "8";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::SPI})));
            temp_gpio_pin_name = "7";
            device.pin_defines.insert(
                std::make_pair(temp_gpio_pin_name,
                               PinDefinition(temp_gpio_pin_name, {PinType::GPIO, PinType::SPI})));
            temp_gpio_pin_name = "5";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "6";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "12";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "13";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "19";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "16";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "26";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "20";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
            temp_gpio_pin_name = "21";
            device.pin_defines.insert(std::make_pair(
                temp_gpio_pin_name, PinDefinition(temp_gpio_pin_name, {PinType::GPIO})));
        }
        return device;
    }
};
#endif