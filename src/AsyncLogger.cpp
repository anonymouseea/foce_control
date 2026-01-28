#include "AsyncLogger.h"
#include <chrono>
#include <iomanip>
#include <sstream>

AsyncLogger::AsyncLogger() : running(true) {
    worker = std::thread(&AsyncLogger::processLogs, this);
}

AsyncLogger::~AsyncLogger() {
    running = false;
    cv.notify_one();
    if (worker.joinable()) worker.join();
    if (logFile.is_open()) {
        logFile.flush();
        logFile.close();
    }
}

void AsyncLogger::setLogFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(mtx);
    if (logFile.is_open()) {
        logFile.flush();
        logFile.close();
    }
    logFile.open(path, std::ios::app);
    useFile = logFile.is_open();
}

static std::string MakeTimestamp() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const auto t = clock::to_time_t(now);
    std::tm tm_val;
    localtime_r(&t, &tm_val);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_val);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << buf << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

void AsyncLogger::log(const std::string& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    logQueue.push(msg);
    cv.notify_one();
}

void AsyncLogger::processLogs() {
    while (running || !logQueue.empty()) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait_for(lock, std::chrono::milliseconds(100), [this] { 
            return !logQueue.empty() || !running; 
        });

        while (!logQueue.empty()) {
            const auto& msg = logQueue.front();
            if (useFile && logFile.is_open()) {
                logFile << '[' << MakeTimestamp() << "] " << msg;
                if (!msg.empty() && msg.back() != '\n') {
                    logFile << '\n';
                }
                logFile.flush();
            }
            logQueue.pop();
        }
    }
}
