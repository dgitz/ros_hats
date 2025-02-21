

#include <gtest/gtest.h>
#include <stdio.h>

#include "../UTMConversion.h"
TEST(BasicTest, TestConversion) {
    // Test Home Coordinates
    double latitude = 40.930362;
    double longitude = -89.765216;

    double northing_m;
    double easting_m;
    char* zone;
    LLtoUTM(23, latitude, longitude, northing_m, easting_m, zone);
    printf("Zone: %c Lat: %f Long: %f Northing: %f Easting: %f\n",
           zone,
           latitude,
           longitude,
           northing_m,
           easting_m);
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
