include(picovision_import.cmake)

add_executable(PACEPicoVision Display.cpp Main.cpp)

target_link_libraries(PACEPicoVision hardware_i2c pico_stdlib aps6404 swd_load)