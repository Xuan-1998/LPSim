#define BOOST_TT_HAS_OPERATOR_HPP_INCLUDED

#include "src/benchmarker.h"
#include "src/linux_host_memory_logger.h"

#ifdef B18_RUN_WITH_GUI
#include <QApplication>
#include "LC_UrbanMain.h"
#else
#include "qcoreapplication.h"
#endif
#include <QDebug>
#include "traffic/b18CommandLineVersion.h"

/*
int find_arg_idx(int argc, char** argv, const char* option) {
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], option) == 0) {
            return i;
        }
    }
    return -1;
}

int find_int_arg(int argc, char** argv, const char* option, int default_value) {
    int iplace = find_arg_idx(argc, argv, option);

    if (iplace >= 0 && iplace < argc - 1) {
        return std::stoi(argv[iplace + 1]);
    }

    return default_value;
}
*/

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
int find_arg_idx(int argc, char** argv, const char* option) {
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], option) == 0) {
            return i;
        }
    }
    return -1;
}

int find_int_arg(int argc, char** argv, const char* option, int default_value) {
    int iplace = find_arg_idx(argc, argv, option);

    if (iplace >= 0 && iplace < argc - 1) {
        return std::stoi(argv[iplace + 1]);
    }

    return default_value;
}}
