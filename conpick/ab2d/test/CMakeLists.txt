find_package(GTest REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/ab2d)

add_executable(base_test base_test.cpp)
target_link_libraries(base_test ab2d GTest::GTest GTest::Main)
add_test(base_test base_test)
