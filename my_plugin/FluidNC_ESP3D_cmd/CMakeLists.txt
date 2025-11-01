add_library(my_plugin INTERFACE)

target_sources(my_plugin INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/fnc_esp3d_cmd.c
)

target_include_directories(my_plugin INTERFACE ${CMAKE_CURRENT_LIST_DIR})
