#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


std::vector<std::string> getI2CBuses() {
    std::vector<std::string> buses;
    std::string cmd = "i2cdetect -l";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        std::cerr << "Failed to execute i2cdetect command" << std::endl;
        return buses;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        std::stringstream ss(buffer);
        std::string bus;
        ss >> bus;
        if (bus.find("i2c-") != std::string::npos) {
            buses.push_back(bus.substr(4));
        }
    }

    int status = pclose(pipe);
    if (status != 0) {
        std::cerr << "i2cdetect command failed with status: " << status << std::endl;
    }

    return buses;
}


std::vector<int> getI2CAddresses(const std::string& bus) {
    std::vector<int> addresses;
    int fd = open((std::string("/dev/") + bus).c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C bus: " << bus << std::endl;
        return addresses;
    }

    for (int addr = 0x03; addr <= 0x77; addr++) {
        if (ioctl(fd, I2C_SLAVE, addr) >= 0) {
            addresses.push_back(addr);
        }
    }

    close(fd);
    return addresses;
}

int main() {
	std::cout << "START" << std::endl;
    std::vector<std::string> buses = getI2CBuses();
    for (const auto& bus : buses) {
        std::cout << "I2C Bus: " << bus << std::endl;
        std::vector<int> addresses = getI2CAddresses(bus);
        for (int addr : addresses) {
            std::cout << "  Address: 0x" << std::hex << addr << std::dec << std::endl;
        }
        std::cout << std::endl;
    }
	std::cout << "END" << std::endl;

    return 0;
}
