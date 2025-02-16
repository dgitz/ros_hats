#include <ros_hats/Channel/GPSInputChannel.h>
GPSInputChannel::~GPSInputChannel() {
}
bool GPSInputChannel::init() {
    bool v = base_init();
    return v;
}
std::string GPSInputChannel::pretty(std::string pre) {
    std::string str = base_pretty(pre);
    str += " Lat: " + std::to_string(position.latitude) +
           " Long: " + std::to_string(position.longitude) +
           " Alt: " + std::to_string(position.altitude) +
           " Update: " + std::to_string(position.update_count);
    return str;
}