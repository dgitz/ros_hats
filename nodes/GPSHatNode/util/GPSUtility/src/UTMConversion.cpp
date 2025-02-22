#include "UTMConversion.h"

#include <math.h>
namespace ros_hats::gps_utility {
UTMConversion::UTMConversion() {
    ellipsoid_map.insert(
        std::pair<std::string, Ellipsoid>("WGS-84", Ellipsoid("WGS-84", 6378137, 0.00669438)));
}
UTMCoordinates UTMConversion::convert(std::string ellipsoid_name, GeograpicCoordinates geo) {
    UTMCoordinates utm;
    auto ellipsoid_it = ellipsoid_map.find(ellipsoid_name);
    if (ellipsoid_it == ellipsoid_map.end()) {
        printf("[ERROR] Ellipsoid: %s is not Supported!\n", ellipsoid_name.c_str());
        return utm;
    }
    // converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
    // East Longitudes are positive, West longitudes are negative.
    // North latitudes are positive, South latitudes are negative
    // Lat and Long are in decimal degrees
    // Written by Chuck Gantz- chuck.gantz@globalstar.com

    double a = ellipsoid_it->second.equatorialRadius;
    double eccSquared = ellipsoid_it->second.eccentricitySquared;
    double k0 = 0.9996;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    // Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (geo.longitude_deg + 180) - int((geo.longitude_deg + 180) / 360) * 360 -
                      180;  // -180.00 .. 179.9;

    double LatRad = geo.latitude_deg * M_PI / 180.0;
    double LongRad = LongTemp * M_PI / 180.0;
    double LongOriginRad;
    int ZoneNumber;

    ZoneNumber = int((LongTemp + 180) / 6) + 1;

    if (geo.longitude_deg >= 56.0 && geo.longitude_deg < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
        ZoneNumber = 32;

    // Special zones for Svalbard
    if (geo.longitude_deg >= 72.0 && geo.longitude_deg < 84.0) {
        if (LongTemp >= 0.0 && LongTemp < 9.0)
            ZoneNumber = 31;
        else if (LongTemp >= 9.0 && LongTemp < 21.0)
            ZoneNumber = 33;
        else if (LongTemp >= 21.0 && LongTemp < 33.0)
            ZoneNumber = 35;
        else if (LongTemp >= 33.0 && LongTemp < 42.0)
            ZoneNumber = 37;
    }
    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone
    LongOriginRad = LongOrigin * M_PI / 180.0;

    // compute the UTM Zone from the latitude and longitude
    utm.utm_zone = std::to_string(ZoneNumber) + compute_zone_letter(geo.longitude_deg);
    eccPrimeSquared = (eccSquared) / (1 - eccSquared);

    N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
    T = tan(LatRad) * tan(LatRad);
    C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
    A = cos(LatRad) * (LongRad - LongOriginRad);

    M = a *
        ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
          5 * eccSquared * eccSquared * eccSquared / 256) *
             LatRad -
         (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
          45 * eccSquared * eccSquared * eccSquared / 1024) *
             sin(2 * LatRad) +
         (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) *
             sin(4 * LatRad) -
         (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));
    utm.easting_m = (double)(k0 * N *
                                 (A + (1 - T + C) * A * A * A / 6 +
                                  (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A *
                                      A * A / 120) +
                             500000.0);

    utm.northing_m =
        (double)(k0 * (M + N * tan(LatRad) *
                               (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                                (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A *
                                    A * A * A * A / 720)));
    if (geo.latitude_deg < 0) {
        utm.northing_m += 10000000.0;  // 10000000 meter offset for southern hemisphere
    }
    return utm;
}
std::vector<std::string> UTMConversion::get_ellipsoids_supported() {
    std::vector<std::string> strs;
    for (auto ellipsoid : ellipsoid_map) { strs.push_back(ellipsoid.first); }

    return strs;
}
std::string UTMConversion::compute_zone_letter(double latitude) {
    if ((84 >= latitude) && (latitude >= 72))
        return "X";
    else if ((72 > latitude) && (latitude >= 64))
        return "W";
    else if ((64 > latitude) && (latitude >= 56))
        return "V";
    else if ((56 > latitude) && (latitude >= 48))
        return "U";
    else if ((48 > latitude) && (latitude >= 40))
        return "T";
    else if ((40 > latitude) && (latitude >= 32))
        return "S";
    else if ((32 > latitude) && (latitude >= 24))
        return "R";
    else if ((24 > latitude) && (latitude >= 16))
        return "Q";
    else if ((16 > latitude) && (latitude >= 8))
        return "P";
    else if ((8 > latitude) && (latitude >= 0))
        return "N";
    else if ((0 > latitude) && (latitude >= -8))
        return "M";
    else if ((-8 > latitude) && (latitude >= -16))
        return "L";
    else if ((-16 > latitude) && (latitude >= -24))
        return "K";
    else if ((-24 > latitude) && (latitude >= -32))
        return "J";
    else if ((-32 > latitude) && (latitude >= -40))
        return "H";
    else if ((-40 > latitude) && (latitude >= -48))
        return "G";
    else if ((-48 > latitude) && (latitude >= -56))
        return "F";
    else if ((-56 > latitude) && (latitude >= -64))
        return "E";
    else if ((-64 > latitude) && (latitude >= -72))
        return "D";
    else if ((-72 > latitude) && (latitude >= -80))
        return "C";
    else
        return "Z";  // This is here as an error flag to show that the latitudeitude is
                     // outside the UTM limits
}
}  // namespace ros_hats::gps_utility