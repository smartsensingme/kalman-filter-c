cmake_minimum_required(VERSION 3.20.0)

if(COMMAND idf_component_register)
    # ESP-IDF Component Registration
    idf_component_register(
        SRCS "kalman.c"
        INCLUDE_DIRS "."
    )
else()
    # Standard CMake (Zephyr, STM32, Desktop, etc.)
    add_library(kalman STATIC kalman.c)
    target_include_directories(kalman PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endif()
