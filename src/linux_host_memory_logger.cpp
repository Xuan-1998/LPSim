#include "linux_host_memory_logger.h"

#include <stdexcept>
#include <cstdlib>
#include <cstdio>
#include <cstring>

const std::string log_file_path = "memory-consumption.csv";

LinuxHostMemoryLogger::LinuxHostMemoryLogger(const int & seconds, const std::string & message) :
    log_thread_(Spawn()),
    cancellation_token_(false),
    delta_(std::chrono::seconds(seconds)),
    message_(message),
    start_(std::chrono::system_clock::now())
{
    log_file_stream_.open(log_file_path);
    if (log_file_stream_.fail()) { cancellation_token_ = true; }
}

void LinuxHostMemoryLogger::LogMemory()
{
    log_file_stream_
        << "timestamp,"
        << "physical-memory-in-KB,"
        << "virtual-memory-in-KB,"
        << "message"
        << std::endl;

    while (!cancellation_token_)
    {
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_);
        log_file_stream_
            << elapsed.count() << ","
            << GetPhysicalMemory() << ","
            << GetVirtualMemory() << ","
            << message_
            << std::endl;
        std::this_thread::sleep_for(delta_);
    }
}

void LinuxHostMemoryLogger::ChangeMessageTo(const std::string & message) { message_ = message; }

std::thread LinuxHostMemoryLogger::Spawn() { return std::thread([this] { this->LogMemory(); }); }

void LinuxHostMemoryLogger::End(){
    cancellation_token_ = true;
    log_thread_.join();
    log_file_stream_.close();
}

int LinuxHostMemoryLogger::ParseLine(char* line)
{
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}

int LinuxHostMemoryLogger::GetVirtualMemory(void)
{
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL)
    {
        if (strncmp(line, "VmSize:", 7) == 0)
        {
            result = ParseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

int LinuxHostMemoryLogger::GetPhysicalMemory(void)
{
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL)
    {
        if (strncmp(line, "VmRSS:", 6) == 0)
        {
            result = ParseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

LinuxHostMemoryLogger memory_logger(1, "Start");
