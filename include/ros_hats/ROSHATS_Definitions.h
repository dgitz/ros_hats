/*! \file ROSHATS_Definitions.h
 */
#ifndef ROSHATS_DEFINITIONS_H
#define ROSHATS_DEFINITIONS_H
#include <eros/eROS_Definitions.h>

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
        UNKNOWN = 0, /*!< Uninitialized value. */
        PWM_HAT = 1, /*!< A Hat that is capable of PWM Input or Output.  This includes Servo Hats.*/
        RELAY_HAT = 2,   /*!< A Hat that is capable of Relay Outputs. */
        ARDUINO_HAT = 3, /*!< A Hat that has an onboard Arduino. */
        END_OF_LIST = 4  /*!< Last item of list. Used for Range Checks. */
    };
    //! Convert HatDefinition::HatType to human readable string
    /*!
      \param v HatDefinition::HatType type
      \return The converted string.
    */
    static std::string HatTypeString(HatDefinition::HatType v) {
        switch (v) {
            case HatDefinition::HatType::UNKNOWN: return "UNKNOWN"; break;
            case HatDefinition::HatType::PWM_HAT: return "PWM_HAT"; break;
            case HatDefinition::HatType::RELAY_HAT: return "RELAY_HAT"; break;
            case HatDefinition::HatType::ARDUINO_HAT: return "ARDUINO_HAT"; break;
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
        PWM = 3, /*!< A PWM Channel is a special type of Digital Channel that only supports values
                    ranging from 0-4096.*/
        END_OF_LIST = 4 /*!< Last item of list. Used for Range Checks. */
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
            case ChannelDefinition::ChannelType::PWM: return "PWM"; break;
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
        else if (v == "PWM") {
            return ChannelDefinition::ChannelType::PWM;
        }
        else {
            return ChannelDefinition::ChannelType::UNKNOWN;
        }
    }

    enum class Direction {
        UNKNOWN = 0, /*!< Uninitialized value. */
        INPUT = 1,   /*!< A Channel that only acts as an Input (relative to the device that is using
                        this Channel). */
        OUTPUT = 2, /*!< A Channel that only acts as an Output (relative to the device that is using
                       this Channel). */
        INPUTOUTPUT = 3, /*!< A Channel that can act as an input or an output (relative to the
                            device that is using this Channel). */
        END_OF_LIST = 4, /*!< Last item of list. Used for Range Checks. */
    };

    //! Convert ChannelDefinition::Direction to human readable string
    /*!
      \param v ChannelDefinition::Direction type
      \return The converted string.
    */
    static std::string ChannelDirectionString(ChannelDefinition::Direction v) {
        switch (v) {
            case ChannelDefinition::Direction::UNKNOWN: return "UNKNOWN"; break;
            case ChannelDefinition::Direction::INPUT: return "INPUT"; break;
            case ChannelDefinition::Direction::OUTPUT: return "OUTPUT"; break;
            case ChannelDefinition::Direction::INPUTOUTPUT: return "INPUTOUTPUT"; break;
            default: return ChannelDirectionString(ChannelDefinition::Direction::UNKNOWN); break;
        }
    }

    static Direction DirectionEnum(std::string v) {
        if (v == "INPUT") {
            return ChannelDefinition::Direction::INPUT;
        }
        else if (v == "OUTPUT") {
            return ChannelDefinition::Direction::OUTPUT;
        }
        else if (v == "INPUTOUTPUT") {
            return ChannelDefinition::Direction::INPUTOUTPUT;
        }
        else {
            ChannelDefinition::Direction::UNKNOWN;
        }
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
struct PWMChannelDataConfig : public ChannelDataConfig {
    PWMChannelDataConfig(int64_t _default_value, int64_t _min_value, int64_t _max_value)
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
    PortConfig(std::string _port_name, ChannelDefinition::ChannelType _port_type)
        : port_name(_port_name), port_type(_port_type) {
    }
    PortConfig(std::string _port_name,
               ChannelDefinition::ChannelType _port_type,
               std::vector<ChannelConfig> _channels)
        : port_name(_port_name), port_type(_port_type), channels(_channels) {
    }
    std::string port_name;
    ChannelDefinition::ChannelType port_type;
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
#endif