/**
 * @file UTMConversion.h
 * @author David Gitz
 * @brief
 * @date 2025-02-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <ros_hats/CoordinateDefinitions.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>
namespace ros_hats {
/**
 * @brief UTMConversion between Geographic and UTM Coordinates
 *
 */
class Ellipsoid
{
   public:
    Ellipsoid(){};

    Ellipsoid(std::string ellipsoidName, double equatorialRadius, double eccentricitySquared)
        : ellipsoidName(ellipsoidName),
          equatorialRadius(equatorialRadius),
          eccentricitySquared(eccentricitySquared) {
    }

    std::string ellipsoidName;
    double equatorialRadius;
    double eccentricitySquared;
};

class UTMConversion
{
   public:
    UTMConversion();
    virtual ~UTMConversion() {
    }
    std::vector<std::string> get_ellipsoids_supported();
    GeograpicCoordinates convert(UTMCoordinates utm);
    UTMCoordinates convert(std::string ellipsoid_name, GeograpicCoordinates geo);
    std::string compute_zone_letter(double latitude);

   private:
    std::map<std::string, Ellipsoid> ellipsoid_map;
};
}  // namespace ros_hats