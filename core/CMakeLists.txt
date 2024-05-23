add_library(PACECore INTERFACE)

target_sources(PACECore INTERFACE
    CPU.cpp
    FloppyController.cpp
    System.cpp
)

target_include_directories(PACECore INTERFACE ${CMAKE_CURRENT_LIST_DIR})