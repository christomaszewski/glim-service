cmake_minimum_required(VERSION 3.22)
project(glim_map_export LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(glim REQUIRED)

# GLIM loads extension modules by library name (config_ros.json extension_modules). The name
# contains "monitor" so glim_ros treats it like its own viewer/monitor modules instead of a
# glim_ext extension (which would log license warnings and look for that package).
add_library(map_export_monitor SHARED src/map_export_monitor.cpp)
target_link_libraries(map_export_monitor glim::glim)
install(TARGETS map_export_monitor LIBRARY DESTINATION lib)

ament_package()
