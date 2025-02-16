/*! \file HatDefinition.h
 */
#pragma once
#include <stdio.h>

#include <boost/bimap.hpp>
#include <map>
#include <memory>
#include <vector>
namespace ros_hats {
/*! \class HatDefinition
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
    static std::string HatTypeString(HatDefinition::HatType v);
    static HatDefinition::HatType HatTypeEnum(std::string v);
};
}  // namespace ros_hats