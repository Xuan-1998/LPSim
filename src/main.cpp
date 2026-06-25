#define BOOST_TT_HAS_OPERATOR_HPP_INCLUDED

#include "lpsim/benchmarker.h"
#include "lpsim/linux_host_memory_logger.h"

#ifdef B18_RUN_WITH_GUI
#include <QApplication>
#else
#include "qcoreapplication.h"
#endif
#include <QDebug>
#include "cli_runner.h"

// NOTE: Check command_line_options for default options.

int main(int argc, char *argv[]) {

  mainBench.startMeasuring();

#ifdef B18_RUN_WITH_GUI
  QApplication a(argc, argv);
  QSettings settings(QApplication::applicationDirPath() +
                     "/command_line_options.ini", QSettings::IniFormat);
  bool useGUI = settings.value("GUI", true).toBool();

  if (useGUI == true) {
    LC::LCUrbanMain w;
    w.showMaximized();
    return a.exec();
  } else {
    LC::B18CommandLineVersion cl;
    cl.runB18Simulation();
    printf(">>Simulation Ended\n");
  }

#else
  QCoreApplication a(argc, argv);
  QSettings settings(QCoreApplication::applicationDirPath() +
                     "/command_line_options.ini", QSettings::IniFormat);

  LC::B18CommandLineVersion cl;
  cl.runB18Simulation();
  printf(">>Simulation Ended\n");
#endif

  mainBench.stopAndEndBenchmark();
  intersectionBench.endBenchmark();
  peopleBench.endBenchmark();
  
  memory_logger.End();
  return 0;
}
