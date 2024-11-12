add_executable(PACEPico2
    Display.cpp
    Main.cpp

    psram.c
)

target_include_directories(PACEPico2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPico2 PACECore PACEPicoShared PACEPicoBIOS hardware_dma pico_stdlib)

pico_enable_stdio_uart(PACEPico2 1)
pico_add_extra_outputs(PACEPico2)