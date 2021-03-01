/*! \file test_definitions.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Channel/Channel.h>
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Channel/PWMInputChannel.h>
#include <ros_hats/Channel/PWMOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/PWMOutputPort.h>
#include <ros_hats/Port/Port.h>
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
#include <sys/time.h>
double time_diff(struct timeval A, struct timeval B) {
    double t1 = A.tv_sec + A.tv_usec / 1000000.0;
    double t2 = B.tv_sec + B.tv_usec / 1000000.0;
    return t2 - t1;
}
TEST(BasicTest, TestBaseOperation) {
    {
        printf("Testing individual Channel Operations...\n");
        std::vector<std::unique_ptr<Channel>> channels;
        channels.emplace_back(new PWMInputChannel("PWMInput0", "P0", 1500, 1000, 2000));
        channels.emplace_back(new PWMOutputChannel("PWMOutput0", "P1", 1500, 1000, 2000));
        channels.emplace_back(new DigitalOutputChannel("DigitalOutput0", "P2"));

        std::size_t passed = 0;
        for (std::size_t i = 0; i < channels.size(); ++i) {
            if (channels.at(i)->get_direction() == ChannelDefinition::Direction::INPUT) {
                if (channels.at(i)->get_channel_type() == ChannelDefinition::ChannelType::PWM) {
                    PWMInputChannel *channel =
                        dynamic_cast<PWMInputChannel *>(channels.at(i).get());
                    if (channel != nullptr) {
                        printf("%s\n", channel->pretty().c_str());
                        passed++;
                    }
                }
            }
            if (channels.at(i)->get_direction() == ChannelDefinition::Direction::OUTPUT) {
                if (channels.at(i)->get_channel_type() == ChannelDefinition::ChannelType::DIGITAL) {
                    DigitalOutputChannel *channel =
                        dynamic_cast<DigitalOutputChannel *>(channels.at(i).get());
                    if (channel != nullptr) {
                        printf("%s\n", channel->pretty().c_str());
                        passed++;
                    }
                }
                if (channels.at(i)->get_channel_type() == ChannelDefinition::ChannelType::PWM) {
                    PWMOutputChannel *channel =
                        dynamic_cast<PWMOutputChannel *>(channels.at(i).get());
                    if (channel != nullptr) {
                        printf("%s\n", channel->pretty().c_str());
                        passed++;
                    }
                }
            }
        }
        EXPECT_TRUE(passed == channels.size());
    }
    {
        printf("Testing Port Operations...\n");
        PWMOutputPort port("PWMOutputPortA", {"P3", "P5", "P7", "P9"});
        EXPECT_TRUE(port.init());
        EXPECT_TRUE(port.get_port_size() == 4);

        ChannelDefinition::ChannelErrorType status = port.update("P3", 1800);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::NOERROR);
        EXPECT_TRUE(port.get_value("P3") == 1800);
        status = port.update("P3", 4000);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND);

        status = port.update("P4", 1500);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);

        uint64_t trials = 20000000;
        struct timeval start, end;
        gettimeofday(&start, NULL);
        printf("Runnig Long duration Timing Tests...\n");
        for (uint64_t i = 0; i < trials; ++i) { status = port.update("P3", 1800); }
        gettimeofday(&end, NULL);
        printf("%s\n", port.pretty().c_str());
        double mtime = time_diff(start, end);
        double avg_time_per_update = mtime / (double)trials;
        printf("Avg time per 1000000 operations: %4.4f(sec)\n", 1000000.0 * avg_time_per_update);
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
