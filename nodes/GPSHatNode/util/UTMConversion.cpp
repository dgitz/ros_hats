#include "UTMConversion.h"
namespace ros_hats {
UTMConversion::UTMConversion() {
    ellipsoid_map.insert(
        std::pair<std::string, Ellipsoid>("Airy", Ellipsoid("Airy", 6377563, 0.00667054)));
    /*

    Ellipsoid(2, "Australian National", 6378160, 0.006694542),
    Ellipsoid(3, "Bessel 1841", 6377397, 0.006674372),
    Ellipsoid(4, "Bessel 1841 (Nambia) ", 6377484, 0.006674372),
    Ellipsoid(5, "Clarke 1866", 6378206, 0.006768658),
    Ellipsoid(6, "Clarke 1880", 6378249, 0.006803511),
    Ellipsoid(7, "Everest", 6377276, 0.006637847),
    Ellipsoid(8, "Fischer 1960 (Mercury) ", 6378166, 0.006693422),
    Ellipsoid(9, "Fischer 1968", 6378150, 0.006693422),
    Ellipsoid(10, "GRS 1967", 6378160, 0.006694605),
    Ellipsoid(11, "GRS 1980", 6378137, 0.00669438),
    Ellipsoid(12, "Helmert 1906", 6378200, 0.006693422),
    Ellipsoid(13, "Hough", 6378270, 0.00672267),
    Ellipsoid(14, "International", 6378388, 0.00672267),
    Ellipsoid(15, "Krassovsky", 6378245, 0.006693422),
    Ellipsoid(16, "Modified Airy", 6377340, 0.00667054),
    Ellipsoid(17, "Modified Everest", 6377304, 0.006637847),
    Ellipsoid(18, "Modified Fischer 1960", 6378155, 0.006693422),
    Ellipsoid(19, "South American 1969", 6378160, 0.006694542),
    Ellipsoid(20, "WGS 60", 6378165, 0.006693422),
    Ellipsoid(21, "WGS 66", 6378145, 0.006694542),
    Ellipsoid(22, "WGS-72", 6378135, 0.006694318),
    Ellipsoid(23, "WGS-84", 6378137, 0.00669438)};
    */
}
GeograpicCoordinates UTMConversion::convert(UTMCoordinates utm) {
    GeograpicCoordinates geo;
    return geo;
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

    double a = ellipsoid_it.second.equatorialRadius;
    double eccSquared = ellipsoid_it.second.eccentricitySquared;
    double k0 = 0.9996;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    // Make sure the longitude is between -180.00 .. 179.9
    double LongTemp =
        (geo.longitude + 180) - int((geo.longitude + 180) / 360) * 360 - 180;  // -180.00 .. 179.9;

    double LatRad = geo.latitude * deg2rad;
    double LongRad = LongTemp * deg2rad;
    double LongOriginRad;
    int ZoneNumber;

    ZoneNumber = int((LongTemp + 180) / 6) + 1;

    if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
        ZoneNumber = 32;

    // Special zones for Svalbard
    if (Lat >= 72.0 && Lat < 84.0) {
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
    LongOriginRad = LongOrigin * deg2rad;

    // compute the UTM Zone from the latitude and longitude
    sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
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
}  // namespace ros_hats