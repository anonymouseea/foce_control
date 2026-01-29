#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <atomic>

#include "AsyncLogger.h"

void RunControlLoop(AsyncLogger& logger, std::atomic<bool>& running);

#endif
