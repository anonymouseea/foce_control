#include "AsyncLogger.h"

AsyncLogger::AsyncLogger() : running(true) {
    worker = std::thread(&AsyncLogger::processLogs, this);
}

AsyncLogger::~AsyncLogger() {
    running = false;
    cv.notify_one();
    if (worker.joinable()) worker.join();
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
            std::cout << logQueue.front() << std::flush;
            logQueue.pop();
        }
    }
}
