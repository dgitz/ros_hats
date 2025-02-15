/*! \file test_definitions.cpp
 */
#include <eros/Logger.h>
#include <gtest/gtest.h>
#include <ros_hats/Channel/Channel.h>
#include <ros_hats/Channel/DigitalInputChannel.h>
#include <ros_hats/Channel/DigitalOutputChannel.h>
#include <ros_hats/Channel/ServoOutputChannel.h>
#include <ros_hats/Hat/Hat.h>
#include <ros_hats/Port/DigitalInputPort.h>
#include <ros_hats/Port/DigitalOutputPort.h>
#include <ros_hats/Port/Port.h>
#include <ros_hats/Port/ServoOutputPort.h>
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
        ChannelConfig servo_output_channel_config;
        servo_output_channel_config.channel_name = "ServoOutput1";
        servo_output_channel_config.channel_type = ChannelDefinition::ChannelType::SERVO;
        servo_output_channel_config.direction = ChannelDefinition::Direction::CH_OUTPUT;
        servo_output_channel_config.pin_number = 0;
        servo_output_channel_config.data_config =
            std::make_shared<ServoChannelDataConfig>(ServoChannelDataConfig(1500, 1000, 2000));

        ChannelConfig digital_output_channel_config;
        digital_output_channel_config.channel_name = "DigitalOutput1";
        digital_output_channel_config.channel_type = ChannelDefinition::ChannelType::DIGITAL;
        digital_output_channel_config.direction = ChannelDefinition::Direction::CH_OUTPUT;
        digital_output_channel_config.pin_number = 0;
        digital_output_channel_config.data_config =
            std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 100));

        ChannelConfig digital_input_channel_config;
        digital_input_channel_config.channel_name = "DigitalInput1";
        digital_input_channel_config.channel_type = ChannelDefinition::ChannelType::DIGITAL;
        digital_input_channel_config.direction = ChannelDefinition::Direction::CH_OUTPUT;
        digital_input_channel_config.pin_number = 1;
        digital_input_channel_config.data_config =
            std::make_shared<DigitalChannelDataConfig>(DigitalChannelDataConfig(0, 0, 100));

        printf("\nTesting individual Channel Operations...\n");
        std::vector<std::shared_ptr<Channel>> channels;
        channels.emplace_back(new ServoOutputChannel(servo_output_channel_config));
        channels.emplace_back(new DigitalOutputChannel(digital_output_channel_config));
        channels.emplace_back(new DigitalInputChannel(digital_input_channel_config));

        std::size_t passed = 0;
        for (std::size_t i = 0; i < channels.size(); ++i) {
            if (channels.at(i)->get_direction() == ChannelDefinition::Direction::CH_OUTPUT) {
                if (channels.at(i)->get_channel_type() == ChannelDefinition::ChannelType::DIGITAL) {
                    DigitalOutputChannel *channel =
                        dynamic_cast<DigitalOutputChannel *>(channels.at(i).get());
                    EXPECT_TRUE(channel != nullptr);
                    if (channel != nullptr) {
                        printf("%s\n", channel->pretty(" ").c_str());
                        passed++;
                    }
                }
                if (channels.at(i)->get_channel_type() == ChannelDefinition::ChannelType::SERVO) {
                    ServoOutputChannel *channel =
                        dynamic_cast<ServoOutputChannel *>(channels.at(i).get());
                    EXPECT_TRUE(channel != nullptr);
                    if (channel != nullptr) {
                        printf("%s\n", channel->pretty(" ").c_str());
                        passed++;
                    }
                }
            }
            else if (channels.at(i)->get_direction() == ChannelDefinition::Direction::CH_INPUT) {
                if (channels.at(i)->get_channel_type() == ChannelDefinition::ChannelType::DIGITAL) {
                    DigitalInputChannel *channel =
                        dynamic_cast<DigitalInputChannel *>(channels.at(i).get());
                    EXPECT_TRUE(channel != nullptr);
                    if (channel != nullptr) {
                        printf("%s\n", channel->pretty(" ").c_str());
                        passed++;
                    }
                }
            }
        }
        EXPECT_TRUE(passed == channels.size());
    }
    {
        printf("\nTesting Port: ServoOutput Operations...\n");
        PortConfig port_config;
        for (int i = 0; i < 4; ++i) {
            ChannelConfig servo_output_channel_config;
            servo_output_channel_config.channel_name = "P" + std::to_string(i);
            servo_output_channel_config.channel_type = ChannelDefinition::ChannelType::SERVO;
            servo_output_channel_config.direction = ChannelDefinition::Direction::CH_OUTPUT;
            servo_output_channel_config.pin_number = i;
            servo_output_channel_config.data_config =
                std::make_shared<ServoChannelDataConfig>(ServoChannelDataConfig(1500, 1000, 2000));
            port_config.channels.push_back(servo_output_channel_config);
        }

        ServoOutputPort port(port_config);
        EXPECT_TRUE(port.init());
        EXPECT_TRUE(port.get_port_size() == 4);
        ChannelDefinition::ChannelErrorType status = port.update("P3", 1800);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::NOERROR);
        printf("%s\n", port.pretty(" ").c_str());
        EXPECT_TRUE(port.get_value("P3") == 1800);
        status = port.update("P3", 4000);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::VALUE_EXCEED_UPPER_BOUND);

        status = port.update("P4", 1500);
        EXPECT_TRUE(status == ChannelDefinition::ChannelErrorType::CHANNEL_NOT_FOUND);

        uint64_t trials = 20000000;
        struct timeval start, end;
        gettimeofday(&start, NULL);
        printf("\nRunning Long duration Timing Tests...\n");
        for (uint64_t i = 0; i < trials; ++i) { status = port.update("P3", 1800); }
        gettimeofday(&end, NULL);
        double mtime = time_diff(start, end);
        double avg_time_per_update = mtime / (double)trials;
        printf("%s\n", port.pretty(" ").c_str());
        printf("(ServoPort) Avg time per 1000000 operations: %4.4f(sec)\n",
               1000000.0 * avg_time_per_update);
    }

    {
        printf("\nTesting Port: DigitalOutput Operations...\n");
        PortConfig port_config;
        for (int i = 0; i < 1; ++i) {
            ChannelConfig digital_output_channel_config;
            digital_output_channel_config.channel_name = "D" + std::to_string(i);
            digital_output_channel_config.channel_type = ChannelDefinition::ChannelType::SERVO;
            digital_output_channel_config.direction = ChannelDefinition::Direction::CH_OUTPUT;
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
        printf("\nRunning Long duration Timing Tests...\n");
        for (uint64_t i = 0; i < trials; ++i) { status = port.update("D0", 50); }
        gettimeofday(&end, NULL);
        printf("%s\n", port.pretty(" ").c_str());
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
