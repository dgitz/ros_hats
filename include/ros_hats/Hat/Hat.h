/*! \file Hat.h
 */
#ifndef ROSHATS_HAT_H
#define ROSHATS_HAT_H
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <ros_hats/Channel/Channel.h>
#include <ros_hats/Port/Port.h>
#include <ros_hats/ROSHATS_Definitions.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

#include "ros/ros.h"
/*! \class Hat
    \brief Hat class
    Basic container for a Hat
*/
class Hat
{
   public:
    Hat();
    ~Hat();
    bool base_init(eros::Logger* _logger, RaspberryPiDefinition::RaspberryPiModel _board);
    virtual bool init_ros(boost::shared_ptr<ros::NodeHandle> n, std::string host_name) = 0;

    std::string base_pretty(std::string pre);
    virtual std::string pretty(std::string pre) = 0;
    eros::Diagnostic::DiagnosticDefinition get_diagnostic() {
        return diagnostic;
    }
    virtual bool update(double dt) = 0;
    virtual bool cleanup() = 0;

   protected:
    boost::shared_ptr<ros::NodeHandle> nodeHandle;
    std::string name;
    eros::Logger* logger;
    RaspberryPiDefinition::RaspberryPi pi_model;
    eros::Diagnostic diag_helper;
    eros::Diagnostic::DiagnosticDefinition diagnostic;
    bool ros_initialized;
};
#endif