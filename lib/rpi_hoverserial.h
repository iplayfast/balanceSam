#ifndef RPI_HOVERSERIAL_H
#define RPI_HOVERSERIAL_H

#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <stdint.h>

// SerialFeedback struct to store the received data
struct SerialFeedback {
    int16_t cmd1;
    int16_t cmd2;
    int16_t speedR_meas;
    int16_t speedL_meas;
    int16_t batVoltage;
    int16_t boardTemp;
    uint16_t cmdLed;
};

class RPiHoverSerial {
private:
    int fd; // File descriptor for the serial port

public:
    RPiHoverSerial() : fd(-1) {}

    bool begin(int baudrate, const char* device = "/dev/ttyAMA0") {
        // Setup wiringPi
        if (wiringPiSetup() == -1) {
            fprintf(stderr, "Unable to start wiringPi: %s\n", strerror(errno));
            return false;
        }
        
        // Open serial device
        fd = serialOpen(device, baudrate);
        if (fd < 0) {
            fprintf(stderr, "Unable to open serial device %s: %s\n", device,strerror(errno));
            return false;
        }
        return true;
    }

    void end() {
        if (fd != -1) {
            serialClose(fd);
            fd = -1;
        }
    }

    void send(int16_t steer, int16_t speed) {
        uint8_t buffer[8];
        buffer[0] = 0xAA;
        buffer[1] = 0xAA;
        buffer[2] = steer >> 8;
        buffer[3] = steer & 0xFF;
        buffer[4] = speed >> 8;
        buffer[5] = speed & 0xFF;
        buffer[6] = 0;
        buffer[7] = 0;

        uint16_t checksum = 0;
        for (int i = 2; i < 8; i++) {
            checksum += buffer[i];
        }

        buffer[6] = checksum >> 8;
        buffer[7] = checksum & 0xFF;

        for (int i = 0; i < 8; i++) {
            serialPutchar(fd, buffer[i]);
        }
    }

    bool receive(SerialFeedback& feedback) {
        uint8_t buffer[10];
        int bytesRead = 0;

        // Wait for start bytes
        while (bytesRead < 2) {
            if (serialDataAvail(fd)) {
                uint8_t c = serialGetchar(fd);
                if (c == 0xAA) {
                    buffer[bytesRead++] = c;
                } else {
                    bytesRead = 0;
                }
            }
        }

        // Read the rest of the frame
        while (bytesRead < 10) {
            if (serialDataAvail(fd)) {
                buffer[bytesRead++] = serialGetchar(fd);
            }
        }

        // Parse the data
        feedback.cmd1 = (buffer[2] << 8) | buffer[3];
        feedback.cmd2 = (buffer[4] << 8) | buffer[5];
        feedback.speedR_meas = (buffer[6] << 8) | buffer[7];
        feedback.speedL_meas = (buffer[8] << 8) | buffer[9];

        // TODO: Add parsing for batVoltage, boardTemp, and cmdLed if needed

        return true;
    }
};

#endif // RPI_HOVERSERIAL_H
