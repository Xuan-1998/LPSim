cmake_minimum_required(VERSION 3.20)
project(LPSim LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

# GPU architecture (A100=80, H100/H200=90, L40=89)
set(CMAKE_CUDA_ARCHITECTURES "80;89;90" CACHE STRING "CUDA architectures")

find_package(CUDAToolkit REQUIRED)
find_package(OpenCV QUIET)
find_package(OpenMP REQUIRED)

# Find Pandana (contraction hierarchies routing library)
find_library(CHROUTING_LIB chrouting PATHS /usr/include/pandana/src /usr/local/lib)
find_path(PANDANA_INCLUDE pandana PATHS /usr/include)

# Qt5 (optional, for legacy compatibility)
find_package(Qt5 COMPONENTS Core Gui QUIET)

# Include paths
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/lpsim
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/simulator
    ${CMAKE_SOURCE_DIR}/src/routing
    ${CMAKE_SOURCE_DIR}/src/routing/sp
    ${CMAKE_SOURCE_DIR}/src/io
    ${CMAKE_SOURCE_DIR}/src/io/RoadGraph
    ${CMAKE_SOURCE_DIR}/src/misctools
    ${PANDANA_INCLUDE}
    /usr/local/boost_1_59_0
    /usr/include
)

# Source files
file(GLOB SIMULATOR_SRC src/simulator/*.cpp)
file(GLOB SIMULATOR_CUDA src/simulator/*.cu)
file(GLOB ROUTING_SRC src/routing/*.cpp src/routing/sp/*.cc)
file(GLOB IO_SRC src/io/*.cpp src/io/RoadGraph/*.cpp)
file(GLOB MISC_SRC src/misctools/*.cpp)

# Main executable
add_executable(lpsim
    src/main.cpp
    src/global.cpp
    ${SIMULATOR_SRC}
    ${ROUTING_SRC}
    ${IO_SRC}
    ${MISC_SRC}
    ${SIMULATOR_CUDA}
)

target_link_libraries(lpsim PRIVATE
    CUDA::cudart
    CUDA::cuda_driver
    OpenMP::OpenMP_CXX
    ${CHROUTING_LIB}
)

if(OpenCV_FOUND)
    target_link_libraries(lpsim PRIVATE ${OpenCV_LIBS})
endif()

if(Qt5_FOUND)
    target_link_libraries(lpsim PRIVATE Qt5::Core Qt5::Gui)
    target_compile_definitions(lpsim PRIVATE QT_CORE_LIB QT_GUI_LIB)
endif()

# CUDA-specific flags
target_compile_options(lpsim PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --use_fast_math
        --expt-relaxed-constexpr
        -Xcompiler=-fopenmp
    >
)

# Install
install(TARGETS lpsim RUNTIME DESTINATION bin)
install(DIRECTORY data/ DESTINATION share/lpsim/data)
install(DIRECTORY include/lpsim/ DESTINATION include/lpsim)

# Output build info
message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
message(STATUS "Pandana library: ${CHROUTING_LIB}")
