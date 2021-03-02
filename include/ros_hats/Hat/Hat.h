/*! \file Hat.h
 */
#ifndef ROSHATS_HAT_H
#define ROSHATS_HAT_H
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <ros_hats/Channel/Channel.h>
#include <ros_hats/Port/Port.h>
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
    bool base_init(Logger* _logger);
    virtual bool init_ros(boost::shared_ptr<ros::NodeHandle> n, std::string host_name) = 0;

    std::string base_pretty();
    virtual std::string pretty() = 0;
    Diagnostic::DiagnosticDefinition get_diagnostic() {
        return diagnostic;
    }

   protected:
    boost::shared_ptr<ros::NodeHandle> nodeHandle;
    std::string name;
    Logger* logger;
    Diagnostic diag_helper;
    Diagnostic::DiagnosticDefinition diagnostic;
};
#endif