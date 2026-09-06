# Homebrew libomp is keg-only; AppleClang does not find OpenMP without these hints.
set(OpenMP_ROOT "/opt/homebrew/opt/libomp" CACHE PATH "Homebrew libomp prefix")
set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include" CACHE STRING "")
set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include" CACHE STRING "")
set(OpenMP_C_LIB_NAMES "omp" CACHE STRING "")
set(OpenMP_CXX_LIB_NAMES "omp" CACHE STRING "")
set(OpenMP_omp_LIBRARY "/opt/homebrew/opt/libomp/lib/libomp.dylib" CACHE FILEPATH "")
