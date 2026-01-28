#ifndef ASYNC_LOGGER_H
#define ASYNC_LOGGER_H

#include <iostream>
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>

class AsyncLogger {
private:
    std::queue<std::string> logQueue;
    std::mutex mtx;
    std::condition_variable cv;
    std::thread worker;
    std::atomic<bool> running;
    std::ofstream logFile;
    bool useFile = false;

    void processLogs(); // 移出函数体

public:
    AsyncLogger();
    ~AsyncLogger();
    void setLogFile(const std::string& path);
    void log(const std::string& msg);
};

#endif