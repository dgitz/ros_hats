#include <ros_hats/Hat/Hat.h>
Hat::Hat() {
}
Hat::~Hat() {
}
std::string Hat::base_pretty() {
    std::string str = "--- Hat Name: " + name;
    return str;
}
bool Hat::base_init(Logger *_logger) {
    if (_logger == nullptr) {
        return false;
    }
    logger = _logger;
    return true;
}