#pragma once

// Include accessibility.h and handle OpenMP macro conflicts
// Pandana defines OpenMP macros that conflict with system OpenMP headers

// First undefine any existing OpenMP macros from Pandana
#ifdef omp_get_thread_num
#undef omp_get_thread_num
#endif
#ifdef omp_get_max_threads
#undef omp_get_max_threads
#endif

// Include Pandana's accessibility.h from its installed location
#include <pandana/src/accessibility.h>

// Now undefine them again to allow proper OpenMP header inclusion
#ifdef omp_get_thread_num
#undef omp_get_thread_num
#endif
#ifdef omp_get_max_threads
#undef omp_get_max_threads
#endif
