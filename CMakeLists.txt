cmake_minimum_required(VERSION 3.5)

if(NOT DEFINED ESP_PLATFORM)

project(scd4x)

include(${PROJECT_SOURCE_DIR}/lib/scd4x.cmake)

add_library(scd4x ${SCD4x_SOURCES})

else()

idf_component_register(
    SRCS "${CMAKE_CURRENT_LIST_DIR}/lib/source/scd4x.c"
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib
)

endif()

