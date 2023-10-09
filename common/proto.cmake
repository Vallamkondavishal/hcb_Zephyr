# Boilerplate code, which pulls in the Zephyr build system.
add_library(hcb_proto INTERFACE)
target_include_directories(hcb_proto INTERFACE ${CMAKE_CURRENT_LIST_DIR}/proto)

message (${CMAKE_CURRENT_LIST_DIR})