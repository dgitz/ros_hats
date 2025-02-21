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
#include <string>
namespace ros_hats {
/**
 * @brief UTMConversion between Geographic and UTM Coordinates
 *
 */
class Ellipsoid
{
   public:
    Ellipsoid(){};

    Ellipsoid(uint8_t id,
              std::string ellipsoidName,
              double equatorialRadius,
              double eccentricitySquared)
        : id(id),
          ellipsoidName(ellipsoidName),
          equatorialRadius(equatorialRadius),
          eccentricitySquared(eccentricitySquared) {
    }

   private:
    uint8_t id;
    std::string ellipsoidName;
    double equatorialRadius;
    double eccentricitySquared;
};

class UTMConversion
{
   public:
    UTMConversion() {
    }
    virtual ~UTMConversion() {
    }
    static GeograpicCoordinates convert(UTMCoordinates utm);
    static UTMCoordinates convert(GeograpicCoordinates geo);
};
}  // namespace ros_hats