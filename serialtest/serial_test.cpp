#include <wiringSerial.h>
#include <iostream>
#include <unistd.h>

int main() {
    const char* device = "/dev/ttyAMA0";
    int baud = 19200;

    int fd = serialOpen(device, baud);
    if (fd < 0) {
        std::cerr << "Error opening " << device << std::endl;
        return 1;
    }

    std::cout << "Opened " << device << " successfully" << std::endl;

    // Simple loopback test
    const char* testMessage = "Hello, Serial!";
    serialPuts(fd, testMessage);
    std::cout << "Sent: " << testMessage << std::endl;

    sleep(1);  // Give some time for the data to be transmitted

    std::cout << "Received: ";
    while (serialDataAvail(fd)) {
        std::cout << (char)serialGetchar(fd);
    }
    std::cout << std::endl;

    serialClose(fd);
    return 0;
}
