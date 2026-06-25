#include "src/benchmarker.h"
#include <QString>


std::ofstream Benchmarker::outStream;
int Benchmarker::amountOpened = 0;
bool Benchmarker::showBenchmarks = false;

void Benchmarker::enableShowBenchmarks(){
  Benchmarker::showBenchmarks = true;
}

Benchmarker::Benchmarker(const std::string desc, const bool print) :
  on(false),
  elapsed(Duration::zero()),
  description(desc),
  printToStdout(print)
{
    if (amountOpened == 0) outStream.open("timestamps.info");
    ++amountOpened;
}

void Benchmarker::startMeasuring()
{
  if (on) return;
  on = true;

  startTimeStamp = std::chrono::high_resolution_clock::now();

  // For more information about epochs please refer to epochconverter.com
  auto timeSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>
                (std::chrono::system_clock::now().time_since_epoch()).count();
    

  if (printToStdout){
    std::cout << "[TIME] Started: <" << description << ">. ";
    if (showBenchmarks){
      std::cout << "Time since epoch (ms): " << timeSinceEpoch;
    }
    std::cout << std::endl;
  }

  auto timeNow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

  return;
}

void Benchmarker::stopMeasuring()
{
  if (!on) return;

  stopTimeStamp = std::chrono::high_resolution_clock::now();

  elapsed += std::chrono::duration_cast<Duration>(
          std::chrono::high_resolution_clock::now() - startTimeStamp);

  on = false;
}

void Benchmarker::endBenchmark()
{
  auto timeSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>
                (std::chrono::system_clock::now().time_since_epoch()).count();

  if (printToStdout){
    std::cout << "[TIME] Ended <"
              << description
              << ">. Elapsed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
              << " ms. ";
    if (showBenchmarks){
      std::cout << "Time since epoch (ms): " << timeSinceEpoch;
    }
    std::cout << std::endl;
  }

  --amountOpened;
  if (amountOpened == 0) outStream.close();
}

void Benchmarker::stopAndEndBenchmark()
{

  stopMeasuring();
  endBenchmark();
}

Benchmarker mainBench("main function");
Benchmarker intersectionBench("Intersections total time");
Benchmarker peopleBench("People total time");
