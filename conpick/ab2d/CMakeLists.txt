aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} BASE_SRCS)
add_library(ab2d STATIC ${BASE_SRCS})

add_subdirectory(test)
