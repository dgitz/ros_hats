/*! \file test_definitions.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Channel/Channel.h>
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Channel/PWMOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/DigitalOutputPort.h>
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
        ChannelConfig pwm_output_channel_config;
        pwm_output_channel_config.channel_name = "PWMOutput1";
        pwm_output_channel_config.channel_type = ChannelDefinition::ChannelType::PWM;
        pwm_output_channel_config.direction = ChannelDefinition::Direction::OUTPUT;
        pwm_output_channel_config.pin_number = 0;
        pwm_output_channel_config.data_config =
            std::make_shared<PWMChannelDataConfig>(PWMChannelDataConfig(1500, 1000, 2000));

        ChannelConfig digital_output_channel_config;
        digital_output_channel_config.channel_name = "DigitalOutput1";
        digital_output_channel_config.channel_type = ChannelDefinition::ChannelType::DIGITAL;
        digital_output_channel_config.direction = ChannelDefinition::Direction::OUTPUT;
        digital_output_channel_config.pin_number = 0;
        digital_output_channel_config.data_config =
            std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 100));
        printf("Testing individual Channel Operations...\n");
        std::vector<std::shared_ptr<Channel>> channels;
        channels.emplace_back(new PWMOutputChannel(pwm_output_channel_config));
        channels.emplace_back(new DigitalOutputChannel(digital_output_channel_config));

        std::size_t passed = 0;
        for (std::size_t i = 0; i < channels.size(); ++i) {
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
        printf("Testing Port: PWMOutput Operations...\n");
        PortConfig port_config;
        for (int i = 0; i < 4; ++i) {
            ChannelConfig pwm_output_channel_config;
            pwm_output_channel_config.channel_name = "P" + std::to_string(i);
            pwm_output_channel_config.channel_type = ChannelDefinition::ChannelType::PWM;
            pwm_output_channel_config.direction = ChannelDefinition::Direction::OUTPUT;
            pwm_output_channel_config.pin_number = i;
            pwm_output_channel_config.data_config =
                std::make_shared<PWMChannelDataConfig>(PWMChannelDataConfig(1500, 1000, 2000));
            port_config.channels.push_back(pwm_output_channel_config);
        }

        PWMOutputPort port(port_config);
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
        double mtime = time_diff(start, end);
        double avg_time_per_update = mtime / (double)trials;
        printf("%s\n", port.pretty().c_str());
        printf("(PWMPort) Avg time per 1000000 operations: %4.4f(sec)\n",
               1000000.0 * avg_time_per_update);
    }

    {
        printf("Testing Port: DigitalOutput Operations...\n");
        PortConfig port_config;
        for (int i = 0; i < 1; ++i) {
            ChannelConfig digital_output_channel_config;
            digital_output_channel_config.channel_name = "D" + std::to_string(i);
            digital_output_channel_config.channel_type = ChannelDefinition::ChannelType::PWM;
            digital_output_channel_config.direction = ChannelDefinition::Direction::OUTPUT;
            digital_output_channel_config.pin_number = i;
            digital_output_channel_config.data_config =
                std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 1));
            port_config.channels.push_back(digital_output_channel_config);
        }
        DigitalOutputPort port(port_config);
        EXPECT_TRUE(port.init());
        EXPECT_TRUE(port.get_port_size() == 1);
        EXPECT_TRUE(port.get_channels().size() == 1);

        ChannelDefinition::ChannelErrorType status = port.update("D0", 0);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::NOERROR);
        EXPECT_TRUE(port.get_value("D0") == 0);
        status = port.update("D0", 2);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND);

        status = port.update("P4", 1);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);

        uint64_t trials = 20000000;
        struct timeval start, end;
        gettimeofday(&start, NULL);
        printf("Runnig Long duration Timing Tests...\n");
        for (uint64_t i = 0; i < trials; ++i) { status = port.update("D0", 50); }
        gettimeofday(&end, NULL);
        printf("%s\n", port.pretty().c_str());
        double mtime = time_diff(start, end);
        double avg_time_per_update = mtime / (double)trials;
        printf("(DigitalPort) Avg time per 1000000 operations: %4.4f(sec)\n",
               1000000.0 * avg_time_per_update);
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
