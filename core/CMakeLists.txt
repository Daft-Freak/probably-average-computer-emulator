add_library(PACECore INTERFACE)

target_sources(PACECore INTERFACE
    CGACard.cpp
    CPU.cpp
    FixedDiskAdapter.cpp
    FloppyController.cpp
    System.cpp
)

target_include_directories(PACECore INTERFACE ${CMAKE_CURRENT_LIST_DIR})