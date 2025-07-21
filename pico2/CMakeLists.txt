add_executable(PACEPico2
    Display.cpp
    Main.cpp

    psram.c
)

# "extra" board config (the thing the pico is plugged into)
if(NOT EXTRA_BOARD)
    if(PICO_BOARD STREQUAL "solderparty_rp2350_stamp_xl")
        set(EXTRA_BOARD stamp_carrier)
    else()
        set(EXTRA_BOARD none)
    endif()
    message("No EXTRA_BOARD set, assuming ${EXTRA_BOARD}")
endif()

string(TOUPPER ${EXTRA_BOARD} EXTRA_BOARD_UPPER)
target_compile_definitions(PACEPico2 PRIVATE EXTRA_BOARD_${EXTRA_BOARD_UPPER}=1)

target_compile_definitions(PACEPico2 PRIVATE PICO_EMBED_XIP_SETUP=1)
target_include_directories(PACEPico2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPico2 PACECore PACEPicoShared PACEPicoBIOS hardware_dma pico_multicore pico_stdlib)

pico_enable_stdio_uart(PACEPico2 1)
pico_add_extra_outputs(PACEPico2)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PACEPico2.uf2
	DESTINATION bin
)