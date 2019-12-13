cmake_minimum_required (VERSION 3.0)
project(update)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (update update.cpp)
target_link_libraries (update Eigen3::Eigen)
