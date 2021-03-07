/*! \file test_gpshat.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Hat/GPSHat.h>
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
#include <sys/time.h>
double time_diff(struct timeval A, struct timeval B) {
    double t1 = A.tv_sec + A.tv_usec / 1000000.0;
    double t2 = B.tv_sec + B.tv_usec / 1000000.0;
    return t2 - t1;
}
TEST(BasicTest, Definitions) {
    printf("All Currently Supported GPS Hats:\n");
    int counter = 0;
    for (uint8_t i = 1; i < (uint8_t)(GPSHat::HatModel::END_OF_LIST); ++i) {
        EXPECT_FALSE(GPSHat::HatModelString((GPSHat::HatModel)(i)) == "UNKNOWN");
        if ((GPSHat::HatModel)(i) != GPSHat::HatModel::UNKNOWN) {
            printf("\t[%d] Supported Hat Model: %s\n",
                   counter,
                   GPSHat::HatModelString((GPSHat::HatModel)(i)).c_str());
            counter++;
        }
    }
}
TEST(BasicTest, TestOperation_GPSHat) {
    printf(
        "NOTE: Executing this test DOES NOT Require to be ran with a Hat Installed.  It SHOULD "
        "Only be ran on a Raspberry Pi.\n");
    {
        printf("Testing Standard Hat.\n");
        Logger* logger = new Logger("DEBUG", "UnitTest_GPSHat");
        GPSHat hat(GPSHat::HatModel::STANDARD);
        HatConfig _config("GPSHat1", "GPSHat", "Standard", true);
        bool status = hat.init(logger, _config);
        printf("%s\n", hat.pretty(" ").c_str());
        if (status == false) {
            logger->log_warn("Hat did not initialize.  Maybe Hat is not installed?  Exiting.\n");
            return;
        }
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);

        double run_time = 10.0;
        double dt = 0.01;
        double timer = 0.0;
        printf("Running GPS Tests for %4.2f Seconds...\n", run_time);
        while (timer < run_time) {
            bool v = hat.update(dt);
            int mod_t = ((int)(timer / dt) % 200);
            if (mod_t == 0) {
                logger->log_info(hat.pretty(" "));
            }
            EXPECT_TRUE(v == true);
            usleep(dt * 1000000.0);
            timer += dt;
        }
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);

        hat.cleanup();
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
