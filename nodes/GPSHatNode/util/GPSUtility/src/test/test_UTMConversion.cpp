

#include <gtest/gtest.h>
#include <stdio.h>

#include "UTMConversion.h"
using namespace ros_hats;
using namespace ros_hats::gps_utility;
TEST(BasicTest, TestDefinitions) {
    UTMConversion SUT;
    EXPECT_GT(SUT.get_ellipsoids_supported().size(), 0);
    // Zone Checks
    double latitude = -80.0;
    double end_latitude = 84.0;
    while (latitude <= end_latitude) {
        std::string zone = SUT.compute_zone_letter(latitude);
        EXPECT_NE(zone, "Z");
        latitude += 0.5;
    }
}
TEST(BasicTest, TestUnsupportedStates) {
    UTMConversion SUT;
    GeograpicCoordinates geo;
    auto utm = SUT.convert("An Ellipsoid that will never be supported", geo);
    EXPECT_EQ(utm.northing_m, 0.0);
    EXPECT_EQ(utm.easting_m, 0.0);
}
TEST(BasicTest, TestConversion) {
    UTMConversion SUT;
    // Test Peoria IL Coordinates
    GeograpicCoordinates geo;
    geo.latitude_deg = 40.693861;
    geo.longitude_deg = -89.589101;

    UTMCoordinates utm_expected;
    utm_expected.easting_m = 281236.968;
    utm_expected.northing_m = 4507997.592;

    UTMCoordinates utm = SUT.convert("WGS-84", geo);

    printf("Lat: %f Long: %f Northing: %f/%f Easting: %f/%f\n",
           geo.latitude_deg,
           geo.longitude_deg,
           utm.northing_m,
           utm_expected.northing_m,
           utm.easting_m,
           utm_expected.easting_m);
    EXPECT_NEAR(utm.northing_m, utm_expected.northing_m, 1e-3);
    EXPECT_NEAR(utm.easting_m, utm_expected.easting_m, 1e-3);
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
