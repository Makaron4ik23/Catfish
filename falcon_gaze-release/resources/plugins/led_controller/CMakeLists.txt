cmake_minimum_required(VERSION 3.16)

project(LedController)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(LedController SHARED
    LedController.cc
)

target_include_directories(LedController PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(LedController
    gz-sim8::gz-sim8
)

install(TARGETS LedController
    LIBRARY DESTINATION lib
)