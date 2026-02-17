add_subdirectory(concurrency)
add_subdirectory(maze)
add_subdirectory(solvers)
add_subdirectory(structures)
add_subdirectory(utils)

add_executable(main main.cpp)

set(THREADS_PREFER_PTHREAD_FLAG TRUE)

add_compile_options(-fsanitize=thread -g)
add_link_options(-fsanitize=thread)


find_package(Threads REQUIRED)

target_link_libraries(main PRIVATE Threads::Threads)

target_link_libraries(main PUBLIC concurrency 
    concurrency
    maze
    solvers
    structures
    utils
)

add_custom_command(
        TARGET main
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/mazes" # Source directory path (from root)
        "$<TARGET_FILE_DIR:main>/mazes" # Destination directory path (relative to the executable's location)
        COMMENT "Copying required directory to build folder"
)


