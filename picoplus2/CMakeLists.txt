add_executable(PACEPicoPlus2
    dvhstx/dvhstx.cpp
    dvhstx/dvi.cpp

    Main.cpp

    psram.c
)

target_include_directories(PACEPicoPlus2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPicoPlus2 PACECore PACEPicoShared PACEPicoBIOS hardware_dma pico_stdlib)

pico_enable_stdio_uart(PACEPicoPlus2 1)
pico_add_extra_outputs(PACEPicoPlus2)