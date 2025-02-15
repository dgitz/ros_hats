/*! \file RaspberryPiDefinition.h
 */
#include <stdio.h>

#include <boost/bimap.hpp>
#include <map>
#include <memory>
#include <vector>
namespace ros_hats {
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
    static RaspberryPiDefinition::RaspberryPiModel RaspberryPiModelFromVersion(std::string v);

    static std::string RaspberryPiModelString(RaspberryPiDefinition::RaspberryPiModel v);
};
}  // namespace ros_hats