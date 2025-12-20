cmake_host_system_information(RESULT NUM_CORES QUERY NUMBER_OF_LOGICAL_CORES)

message(STATUS "Number of logical cores detected: ${NUM_CORES}")

add_library(concurrency INTERFACE)

target_include_directories(concurrency INTERFACE .)
target_compile_definitions(concurrency INTERFACE  "MAX_THREADS=${NUM_CORES}")
