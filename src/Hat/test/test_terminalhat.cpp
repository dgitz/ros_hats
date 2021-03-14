/*! \file test_terminalhat.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Hat/TerminalHat.h>
#include <ros_hats/ROSHATS_Definitions.h>
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
        Logger* logger = new Logger("DEBUG", "UnitTest_TerminalHat");
        TerminalHat hat(TerminalHat::HatModel::STANDARD);
        HatConfig _config("TerminalHat1", "TerminalHat", "Standard", false);
        PortConfig digitalinput_port = PortConfig("DigitalInputPort",
                                                  ChannelDefinition::ChannelType::DIGITAL,
                                                  ChannelDefinition::Direction::CH_INPUT);
        ChannelConfig pps_channel = ChannelConfig("PPSInput",
                                                  ChannelDefinition::ChannelType::DIGITAL,
                                                  ChannelDefinition::Direction::CH_INPUT,
                                                  7);  // GPSHat hard-wired to GPIO04.
        pps_channel.data_config =
            std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 1));
        digitalinput_port.channels.push_back(pps_channel);

        _config.ports.push_back(digitalinput_port);
        EXPECT_TRUE(hat.init(
            logger, RaspberryPiDefinition::RaspberryPiModel::RASPBERRYPI_4_MODEL_B, _config));
        EXPECT_TRUE(hat.get_board().pin_map.size() > 0);
        EXPECT_TRUE(hat.get_board().pin_map.size() == hat.get_board().pin_defines.size());
        printf("%s\n", hat.pretty(" ").c_str());
        EXPECT_TRUE(hat.get_diagnostic().level <= Level::Type::NOTICE);

        double run_time = 60.0;
        double dt = 0.01;
        double timer = 0.0;
        while (timer < run_time) {
            bool v = hat.update(dt);
            int mod_t = ((int)(timer / dt) % 200);
            if (mod_t == 0) {
                printf("%s\n", hat.pretty(" ").c_str());
            }
            EXPECT_TRUE(v == true);
            usleep(dt * 1000000.0);
            timer += dt;
        }
        hat.cleanup();
        delete logger;
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
