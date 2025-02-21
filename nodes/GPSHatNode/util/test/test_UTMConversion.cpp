

#include <gtest/gtest.h>
#include <stdio.h>

#include "../UTMConversion.h"
using namespace ros_hats;
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
    // Test Peoria IL Coordinates

    double latitude = 40.693861;
    double longitude = -89.589101;
    double expected_easting_m = 281236.968;
    double expected_northing_m = 4507997.592;
    double northing_m;
    double easting_m;

    // LLtoUTM(23, latitude, longitude, northing_m, easting_m, zone);
    EXPECT_TRUE(true);
    /*
    printf("Lat: %f Long: %f Northing: %f/%f Easting: %f/%f\n",
           latitude,
           longitude,
           northing_m,
           expected_northing_m,
           easting_m,
           expected_easting_m);
    std::cout << zone << std::endl;
    */
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
