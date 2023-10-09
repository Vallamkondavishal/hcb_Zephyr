# Boilerplate code, which pulls in the Zephyr build system.
cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD hcb)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(HCB VERSION 0.1 LANGUAGES C)
message(STATUS "HCB Version: ${PROJECT_VERSION} ") 

set (COMMON_PATH ${CMAKE_CURRENT_LIST_DIR}/common)
include(${COMMON_PATH}/proto.cmake)

add_library(project_options INTERFACE)
target_compile_options(project_options INTERFACE -fdiagnostics-color=always -Wno-sign-compare -Wall -Wextra -Wnull-dereference -Wmisleading-indentation -Wduplicated-cond -Wno-unused-parameter -Werror=return-type)
target_compile_definitions(project_options  INTERFACE "HCB_FW_VERSION=\"${PROJECT_VERSION}\"")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Add your source file to the "app" target. This must come after
# the boilerplate code, which defines the target.

set(APP_SRC 
src/main.c 
src/adc_subsystem.h src/adc_subsystem.c
src/usb_hcb_subsystem.h src/usb_hcb_subsystem.c
src/can_subsystem.h src/can_subsystem.c
)

target_sources(app PRIVATE ${APP_SRC})

target_link_libraries(app PUBLIC hcb_proto project_options)
