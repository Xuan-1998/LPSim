#ifndef LINUX_MEMOTY_LOGGER__H
#define LINUX_MEMOTY_LOGGER__H


#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <atomic>

class LinuxHostMemoryLogger
{
    using Secs = std::chrono::duration<double, std::ratio<1>>;

    public:
        LinuxHostMemoryLogger(const int & seconds, const std::string & message);
        void End();
        void ChangeMessageTo(const std::string & message);


    private:
        std::thread log_thread_;
        std::atomic_bool cancellation_token_;
        const Secs delta_;
        std::string message_;
        const std::chrono::time_point<std::chrono::system_clock> start_;
        std::ofstream log_file_stream_;

        void LogMemory(void);
        std::thread Spawn(void);
        int ParseLine(char* line);
        int GetPhysicalMemory(void);
        int GetVirtualMemory(void);
};

extern LinuxHostMemoryLogger memory_logger;


#endif  // LINUX_MEMOTY_LOGGER__H

