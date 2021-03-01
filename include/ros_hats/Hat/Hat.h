/*! \file Hat.h
 */
#ifndef ROSHATS_HAT_H
#define ROSHATS_HAT_H
#include <ros_hats/Channel/Channel.h>
#include <ros_hats/Port/Port.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class Hat
    \brief Hat class
    Basic container for a Hat
*/
class Hat
{
   public:
    Hat();
    ~Hat();
    bool init();

    std::string pretty();

   private:
};
#endif