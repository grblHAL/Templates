add_library(hpgl INTERFACE)

target_sources(hpgl INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/motori.c
 ${CMAKE_CURRENT_LIST_DIR}/hpgl.c
 ${CMAKE_CURRENT_LIST_DIR}/arc.c
 ${CMAKE_CURRENT_LIST_DIR}/clip.c
 ${CMAKE_CURRENT_LIST_DIR}/charset0.c
 ${CMAKE_CURRENT_LIST_DIR}/font173.c
 ${CMAKE_CURRENT_LIST_DIR}/htext.c
 ${CMAKE_CURRENT_LIST_DIR}/scale.c
)

target_include_directories(hpgl INTERFACE ${CMAKE_CURRENT_LIST_DIR})
