project(hidtest)

cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

set (COMMON_PATH ${CMAKE_CURRENT_LIST_DIR}/../common)

# Prefer pthread over system thread - DEVONO stare prima del find_package!
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

#Sets the C++ standard
set(CMAKE_CXX_STANDARD 17)


include(cmake/StandardProjectSettings.cmake)
include(cmake/CompilerWarnings.cmake)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

include(${COMMON_PATH}/proto.cmake)

# Option needed to obtain coloring error in Ninja
add_library(hcbapi hcbapi.h hcbapi.c)
target_link_libraries(hcbapi PUBLIC m udev hcb_proto hidapi-libusb)
set_property(TARGET hcbapi PROPERTY C_STANDARD 99)
set_project_warnings(hcbapi)


add_executable(hcbcli hcbcli.cc)
target_link_libraries(hcbcli PUBLIC hcbapi Threads::Threads)
set_project_warnings(hcbcli)

add_executable(hcbmon hcbmonitor.cpp)
target_link_libraries(hcbmon PUBLIC hcbapi Threads::Threads ncurses)