/*! \file test_HatNodeProcess.cpp
 */
#include <gtest/gtest.h>
#include <ros_hats/nodes/HatNode/HatNodeProcess.h>
#include <stdio.h>
using namespace eros;

static std::string get_hostname() {
    char name[1024];
    name[1023] = '\0';
    gethostname(name, 1023);
    return std::string(name);
}
class HatNodeProcessTester : public HatNodeProcess
{
   public:
    HatNodeProcessTester() {
    }
    ~HatNodeProcessTester() {
    }
};
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestHatNodeProcess");
    HatNodeProcessTester* tester = new HatNodeProcessTester;
    tester->initialize("UnitTestHatNodeProcess",
                       "UnitTestHatNodeProcess",
                       "ControlModule2",
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::COMMUNICATIONS);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    std::map<std::string, HatConfig> hat_configs =
        tester->load_hat_config("/home/robot/config/DeviceList.json");
    printf("%s\n", HatNodeProcess::pretty(hat_configs).c_str());
    EXPECT_TRUE(hat_configs.size() > 0);
    EXPECT_TRUE(hat_configs.begin()->second.ports.size() > 0);
    EXPECT_TRUE(hat_configs.begin()->second.ports.at(0).channels.size() > 0);
    delete logger;
    delete tester;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
