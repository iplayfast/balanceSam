#ifndef PID_CONF_H
#define PID_CONF_H
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <unordered_map>
#include "pid.h"

class PIDConfigManager {
public:
    PIDConfigManager(const std::string& filename, PID& pid)
        : filename(filename), pid(pid), lastModified(std::filesystem::last_write_time(filename)) {
        loadConfig();
    }

    void watchForChanges() {
        while (true) {
            auto currentModified = std::filesystem::last_write_time(filename);
            if (currentModified != lastModified) {
                loadConfig();
                lastModified = currentModified;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

private:
    void loadConfig() {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << filename << std::endl;
            return;
        }

        std::unordered_map<std::string, double> config;
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            double value;
            if (std::getline(iss, key, '=') && iss >> value) {
                config[key] = value;
            }
        }

        if (config.count("Kp") && config.count("Ki") && config.count("Kd")) {
            pid.setTunings(config["Kp"], config["Ki"], config["Kd"]);
        }
        if (config.count("setpoint")) {
            *pid.getSetpoint() = config["setpoint"];
        }

        std::cout << "Updated PID configuration: "
                  << "Kp=" << config["Kp"] << ", Ki=" << config["Ki"] << ", Kd=" << config["Kd"]
                  << ", Setpoint=" << config["setpoint"] << std::endl;
    }

    std::string filename;
    PID& pid;
    std::filesystem::file_time_type lastModified;
};

#endif // PID_CONF_H

