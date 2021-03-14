#include <ros_hats/Hat/Driver/GPIODriver.h>
GPIODriver::GPIODriver() {
}

GPIODriver::~GPIODriver() {
}
void GPIODriver::inputCB() {
    printf("input: \n");
}
bool GPIODriver::init(std::string _gpio_pin_name,
                      ChannelDefinition::ChannelType _channel_type,
                      ChannelDefinition::Direction _channel_direction) {
    gpio_pin_name = _gpio_pin_name;
    channel_type = _channel_type;
    channel_direction = _channel_direction;
    pin_number = std::atoi(gpio_pin_name.c_str());
    number = pin_number;
    wiringPiSetupGpio();
    /*
    // TODO export pin, set mode of pin, callback register, etc.
    if (export_gpio(gpio_pin_name) == false) {
        return false;
    }
    if (channel_type == ChannelDefinition::ChannelType::DIGITAL) {
        if (channel_direction == ChannelDefinition::Direction::CH_INPUT) {
            if (setdir_gpio(gpio_pin_name, "in") == false) {
                return false;
            }
        }
    }
    */
    if (channel_type == ChannelDefinition::ChannelType::DIGITAL) {
        if (channel_direction == ChannelDefinition::Direction::CH_INPUT) {
            pinMode(pin_number, INPUT);
            printf("Creating Input for Pin: %d\n", pin_number);
            wiringPiISR(pin_number, INT_EDGE_RISING, inputCB);
        }
    }
    return true;
}
bool GPIODriver::cleanup() {
    // TODO Unexport Pin, set to input, etc.
    /*
    if (unexport_gpio(gpio_pin_name) == false) {
        return false;
    }
    */
    return true;
}

bool GPIODriver::export_gpio(std::string pin_name) {
    std::string export_str = "/sys/class/gpio/export";
    std::ofstream export_gpio_fd(export_str.c_str());
    if (export_gpio_fd.is_open() == false) {
        return false;
    }
    export_gpio_fd << pin_name;
    export_gpio_fd.close();
    usleep(DELAY_GPIOEXPORT_MS);
    return true;
}
bool GPIODriver::unexport_gpio(std::string pin_name) {
    std::string unexport_str = "/sys/class/gpio/unexport";
    std::ofstream unexport_gpio_fd(unexport_str.c_str());
    if (unexport_gpio_fd.is_open() == false) {
        return false;
    }

    unexport_gpio_fd << pin_name;
    unexport_gpio_fd.close();
    usleep(DELAY_GPIOEXPORT_MS);
    return true;
}
bool GPIODriver::setdir_gpio(std::string pin_name, std::string dir) {
    std::string out_str = "/sys/class/gpio/gpio" + pin_name + "/direction";
    std::ofstream setdir_gpio_fd(out_str.c_str());
    if (setdir_gpio_fd.is_open() == false) {
        return false;
    }
    setdir_gpio_fd << dir;
    setdir_gpio_fd.close();
    return true;
}
bool GPIODriver::setvalue_gpio(std::string pin_name, std::string value) {
    std::string out_str = "/sys/class/gpio/gpio" + pin_name + "/value";
    std::ofstream setvalue_gpio_fd(out_str.c_str());
    if (setvalue_gpio_fd.is_open() == false) {
        return false;
    }
    setvalue_gpio_fd << value;
    setvalue_gpio_fd.close();
    return true;
}