execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_CURRENT_SOURCE_DIR}/Data
  ${CMAKE_CURRENT_BINARY_DIR}/Data)


# Define the framework files.
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Framework Testbed_Framework_SRCS)

include_directories (
	${Box2D_SOURCE_DIR}
)

add_executable(sim
	${Testbed_Framework_SRCS}
	sim.cpp
)

target_link_libraries (
	sim
	Box2D
        imgui
        ${GLFW_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${OPENGL_LIBRARIES}
)
