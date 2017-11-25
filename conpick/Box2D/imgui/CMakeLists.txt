aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} IMGUI_SRCS)
add_library(imgui ${IMGUI_SRCS})
target_link_libraries(imgui ${GLFW_LIBRARIES})
