/*! \file RaspberryPiDefinition.cpp
 */
#include <ros_hats/RaspberryPiDefinition.h>
#include <stdio.h>

#include <boost/bimap.hpp>
#include <map>
#include <memory>
#include <vector>
namespace ros_hats {

RaspberryPiDefinition::RaspberryPiModel RaspberryPiDefinition::RaspberryPiModelFromVersion(
    std::string v) {
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
std::string RaspberryPiDefinition::RaspberryPiModelString(
    RaspberryPiDefinition::RaspberryPiModel v) {
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
}  // namespace ros_hats