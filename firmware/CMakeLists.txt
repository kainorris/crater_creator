# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(__PROJECT_GROUP_LINK_COMPONENTS ON)


list(APPEND EXTRA_COMPONENT_DIRS components/ESP32-VL53L0X)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(VL53L1-esp-idf)
