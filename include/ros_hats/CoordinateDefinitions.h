/**
 * @file UTMDefinitions.h
 * @author David Gitz
 * @brief
 * @date 2025-02-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
namespace ros_hats {
struct GeograpicCoordinates {
    double latitude{0.0};
    double longitude{0.0};
};
struct UTMCoordinates {
    double northing_m{0.0};
    double easting_m{0.0};
    std::string utm_zone{""};
};
}  // namespace ros_hats