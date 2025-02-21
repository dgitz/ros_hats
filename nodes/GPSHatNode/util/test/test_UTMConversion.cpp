

#include <gtest/gtest.h>
#include <stdio.h>

#include "../UTMConversion.h"
TEST(BasicTest, TestConversion) {
    // Test Peoria IL Coordinates

    double latitude = 40.693861;
    double longitude = -89.589101;
    double expected_easting_m = 281236.968;
    double expected_northing_m = 4507997.592;
    double northing_m;
    double easting_m;

    // LLtoUTM(23, latitude, longitude, northing_m, easting_m, zone);
    EXPECT_TRUE(false);
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
