cmake_minimum_required (VERSION 3.0)
project(nav_test)

include_directories(include)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(nav_test nav_test.cpp)
target_link_libraries(nav_test Eigen3::Eigen)
