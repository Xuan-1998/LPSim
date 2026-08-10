cmake_minimum_required(VERSION 3.20)

# Remember whether the caller supplied CMake's standard CUDA architecture
# setting. project() asks nvcc for a host-specific default, which we replace
# below only when the caller did not make an explicit choice.
if(DEFINED CMAKE_CUDA_ARCHITECTURES OR
   (DEFINED ENV{CUDAARCHS} AND NOT "$ENV{CUDAARCHS}" STREQUAL ""))
    set(LPSIM_CUDA_ARCHITECTURES_EXPLICIT TRUE)
else()
    set(LPSIM_CUDA_ARCHITECTURES_EXPLICIT FALSE)
endif()

project(LPSim LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

# Optional project-specific override. The standard
# -DCMAKE_CUDA_ARCHITECTURES option remains fully supported.
set(LPSIM_CUDA_ARCHITECTURES "" CACHE STRING
    "Override LPSim CUDA architectures (for example: 90-real;90-virtual)")

if(LPSIM_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES "${LPSIM_CUDA_ARCHITECTURES}" CACHE STRING
        "CUDA architectures" FORCE)
    set(LPSIM_CUDA_ARCHITECTURE_PROFILE "custom")
elseif(NOT LPSIM_CUDA_ARCHITECTURES_EXPLICIT)
    if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 12.8)
        # Native cubins: A100, L40, H100/H200, B100/B200.
        # PTX: Ampere baseline plus Blackwell for current and future drivers.
        set(CMAKE_CUDA_ARCHITECTURES
            "80-real;89-real;90-real;100-real;80-virtual;100-virtual"
            CACHE STRING "CUDA architectures" FORCE)
        set(LPSIM_CUDA_ARCHITECTURE_PROFILE "portable-native")
    else()
        # CUDA < 12.8 cannot emit an sm_100 cubin. Keep PTX so a sufficiently
        # new NVIDIA driver can JIT the kernels on Blackwell GPUs.
        set(CMAKE_CUDA_ARCHITECTURES
            "80-real;89-real;90-real;80-virtual;90-virtual"
            CACHE STRING "CUDA architectures" FORCE)
        set(LPSIM_CUDA_ARCHITECTURE_PROFILE "portable-jit")
        message(WARNING
            "CUDA ${CMAKE_CUDA_COMPILER_VERSION} cannot emit native Blackwell "
            "(sm_100) code; B100/B200 will use the embedded PTX fallback. "
            "Use CUDA 12.8 or newer for a native Blackwell cubin.")
    endif()
else()
    set(LPSIM_CUDA_ARCHITECTURE_PROFILE "custom")
endif()

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
file(GLOB ROUTING_SRC src/routing/*.cpp src/routing/sp/graph.cc)
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

string(REPLACE ";" "," LPSIM_CUDA_ARCHITECTURES_TEXT
    "${CMAKE_CUDA_ARCHITECTURES}")
target_compile_definitions(lpsim PRIVATE
    LPSIM_CUDA_ARCHITECTURES="${LPSIM_CUDA_ARCHITECTURES_TEXT}"
    LPSIM_CUDA_COMPILER_VERSION="${CMAKE_CUDA_COMPILER_VERSION}"
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
message(STATUS "CUDA compiler: ${CMAKE_CUDA_COMPILER_VERSION}")
message(STATUS "CUDA architecture profile: ${LPSIM_CUDA_ARCHITECTURE_PROFILE}")
message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
message(STATUS "Pandana library: ${CHROUTING_LIB}")
