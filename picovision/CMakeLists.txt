include(picovision_import.cmake)

add_executable(PACEPicoVision
    Display.cpp
    Filesystem.cpp
    Main.cpp
    Storage.cpp
)

target_include_directories(PACEPicoVision PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPicoVision PACECore PACEPicoShared PACEPicoBIOS hardware_i2c pico_stdlib aps6404 swd_load)

pico_generate_pio_header(PACEPicoVision ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

pico_add_extra_outputs(PACEPicoVision)