/*! \file ROSHATS_Definitions.h
 */
#ifndef ROSHATS_DEFINITIONS_H
#define ROSHATS_DEFINITIONS_H
#include <eros/eROS_Definitions.h>

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
};
#endif