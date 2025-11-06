# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set custom partition table for hexapod configuration
set(PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")
set(PARTITION_TABLE_CSV_PATH "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hexapod-robot)
