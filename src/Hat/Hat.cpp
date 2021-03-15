#include <ros_hats/Hat/Hat.h>
Hat::Hat() : ros_initialized(false) {
}
Hat::~Hat() {
}
std::string Hat::base_pretty(std::string pre) {
    std::string str = pre + "Hat: " + name;
    return str;
}
bool Hat::base_init(Logger *_logger, RaspberryPiDefinition::RaspberryPiModel _board) {
    if (_logger == nullptr) {
        return false;
    }
    logger = _logger;
    pi_model = RaspberryPiDefinition::build_RaspberryPi(_board);
    return true;
}