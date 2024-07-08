add_library(PACECore INTERFACE)

target_sources(PACECore INTERFACE
    AboveBoard.cpp
    CGACard.cpp
    CPU.cpp
    FixedDiskAdapter.cpp
    FloppyController.cpp
    SerialMouse.cpp
    System.cpp
)

target_include_directories(PACECore INTERFACE ${CMAKE_CURRENT_LIST_DIR})