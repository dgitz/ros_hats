/*! \file GPSInputChannel.h
 */
#ifndef ROSHATS_GPSINPUT_CHANNEL_H
#define ROSHATS_GPSINPUT_CHANNEL_H
#include <ros_hats/Channel/Channel.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

/*! \class GPSInputChannel
    \brief GPSInputChannel class
    Basic container for a GPSInputChannel
*/
class GPSInputChannel : public Channel
{
   public:
    enum class FixType {
        UNKNOWN = 0,
        NO_FIX = 1,
        FIX_GPS = 2,
        FIX_DGPS = 3,
        END_OF_LIST = 4,
    };
    struct Position {
        Position() : timestamp(-1.0), update_count(0) {
        }
        double timestamp;
        double latitude;
        double longitude;
        double altitude;
        uint64_t update_count;
    };
    struct Status {
        Status() : timestamp(-1.0), update_count(0) {
        }
        double timestamp;
        FixType fix_type;
        uint16_t satellites_visible;
        uint64_t update_count;
    };
    GPSInputChannel() {
    }
    GPSInputChannel(ChannelConfig _config) : update_count(0) {
        channel_config = _config;
        auto data_config = std::static_pointer_cast<GPSChannelDataConfig>(_config.data_config);
        channel_config.channel_type = ChannelDefinition::ChannelType::GPS;
        channel_config.direction = ChannelDefinition::Direction::INPUT;
    }
    ~GPSInputChannel();

    bool init();

    std::string pretty(std::string pre);

    ChannelDefinition::ChannelErrorType update_value(Position _position, Status _status) {
        position = _position;
        status = _status;
        update_count++;
        return ChannelDefinition::ChannelErrorType::NOERROR;
    }
    Position get_position() {
        return position;
    }
    Status get_status() {
        return status;
    }

   private:
    uint64_t update_count;
    Status status;
    Position position;
};
#endif  // ROSHATS_GPSINPUT_CHANNEL_H