#pragma once
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <atomic> // 1. Include atomic for thread-safe reading of the level
#include <algorithm>
#include "json.hpp"

namespace Logger {

    //(DEBUG < INFO < WARN < ERROR)
    enum class LogLevel {
        ERROR,
        WARN,
        INFO,
        DEBUG,
        NONE
    };

    inline std::string to_string(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO";
            case LogLevel::WARN:  return "WARN";
            case LogLevel::ERROR: return "ERROR";
            default:              return "NONE";
        }
    }

    inline LogLevel parse_level(std::string str) {
        std::transform(str.begin(), str.end(), str.begin(), ::toupper);
        if (str == "INFO") return LogLevel::INFO; 
        if (str == "DEBUG") return LogLevel::DEBUG;
        if (str == "WARN" || str == "WARNING")  return LogLevel::WARN;
        if (str == "ERROR" || str == "ERR")   return LogLevel::ERROR;
        return LogLevel::NONE; // Default fallback
    }

    inline std::mutex log_mutex;
    inline std::atomic<LogLevel> active_level{LogLevel::INFO};

    // Function to set the active log level
    inline void set_level(LogLevel level) {
        active_level.store(level);
    }

    //ISO 8601 UTC timestamp
    inline std::string get_timestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&in_time_t), "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }

    //Logger function
    inline void log(LogLevel level, const std::string& event, const nlohmann::json& extra_fields = nlohmann::json::object()) {
        if (static_cast<int>(level) > static_cast<int>(active_level.load())) {
            return; 
        }

        nlohmann::json log_payload = {
            {"timestamp", get_timestamp()},
            {"level", to_string(level)},
            {"event", event}
        };

        log_payload.update(extra_fields);

        std::lock_guard<std::mutex> lock(log_mutex);
        std::cout << log_payload.dump() << std::endl;
    }

    //Helpers
    inline void debug(const std::string& event, const nlohmann::json& extra = nlohmann::json::object()) {
        log(LogLevel::DEBUG, event, extra);
    }
    inline void info(const std::string& event, const nlohmann::json& extra = nlohmann::json::object()) {
        log(LogLevel::INFO, event, extra);
    }
    inline void warn(const std::string& event, const nlohmann::json& extra = nlohmann::json::object()) {
        log(LogLevel::WARN, event, extra);
    }
    inline void error(const std::string& event, const nlohmann::json& extra = nlohmann::json::object()) {
        log(LogLevel::ERROR, event, extra);
    }
}