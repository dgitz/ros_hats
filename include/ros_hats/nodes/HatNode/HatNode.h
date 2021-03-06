/*! \file HatNode.h
 */
#ifndef HatNode_H
#define HatNode_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>
#include <ros_hats/Hat/Hat.h>
#ifdef __arm__
#include <ros_hats/Hat/GPSHat.h>
#include <ros_hats/Hat/RelayHat.h>
#include <ros_hats/Hat/ServoHat.h>
#include <ros_hats/Hat/TerminalHat.h>
#endif

#include <ros_hats/nodes/HatNode/HatNodeProcess.h>
/*! \class HatNode HatNode.h "HatNode.h"
 *  \brief */
class HatNode : public eros::BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "hat_node";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 1;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 2;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 23-March-2021";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ROBOT_CONTROLLER;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::CONTROLLER;
    HatNode();
    ~HatNode();
    HatNodeProcess* get_process() {
        return process;
    }
    bool start();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    bool run_loop1();
    bool run_loop2();
    bool run_loop3();
    bool run_001hz();
    bool run_01hz();
    bool run_01hz_noisy();
    bool run_1hz();
    bool run_10hz();
    void thread_loop();
    void cleanup();

    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);

   private:
    struct hat_config {
        std::string hat_name;
        std::string hat_type;
        std::string hat_model;
    };
    eros::Diagnostic::DiagnosticDefinition read_launchparameters();
    HatNodeProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;

    std::map<std::string, std::shared_ptr<Hat>> hats;
};

#endif  // HatNode_H
