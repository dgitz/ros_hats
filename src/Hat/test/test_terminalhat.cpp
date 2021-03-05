/*! \file test_terminalhat.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Hat/TerminalHat.h>
#include <stdio.h>
#include <sys/time.h>
double time_diff(struct timeval A, struct timeval B) {
    double t1 = A.tv_sec + A.tv_usec / 1000000.0;
    double t2 = B.tv_sec + B.tv_usec / 1000000.0;
    return t2 - t1;
}
TEST(BasicTest, Definitions) {
    printf("All Currently Supported Relay Hats:\n");
    int counter = 0;
    for (uint8_t i = 1; i < (uint8_t)(TerminalHat::HatModel::END_OF_LIST); ++i) {
        EXPECT_FALSE(TerminalHat::HatModelString((TerminalHat::HatModel)(i)) == "UNKNOWN");
        if ((TerminalHat::HatModel)(i) != TerminalHat::HatModel::UNKNOWN) {
            printf("\t[%d] Supported Hat Model: %s\n",
                   counter,
                   TerminalHat::HatModelString((TerminalHat::HatModel)(i)).c_str());
            counter++;
        }
    }
}
TEST(BasicTest, TestOperation_TerminalHat) {
    printf(
        "NOTE: Executing this test DOES NOT Require to be ran with a Hat Installed.  It SHOULD "
        "Only be ran on a Raspberry Pi.\n");

    {
        printf(
            "Testing Terminal Hat.  This Test assumes the default configuration of jumpers "
            "installed.\n");
        Logger* logger = new Logger("DEBUG", "UnitTest_TerminalHat");
        TerminalHat hat(TerminalHat::HatModel::GENERIC);
        HatConfig _config("TerminalHat1", "TerminalHat", "GENERIC", true);
        EXPECT_TRUE(hat.init(logger, _config));
        printf("%s\n", hat.pretty(" ").c_str());
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
