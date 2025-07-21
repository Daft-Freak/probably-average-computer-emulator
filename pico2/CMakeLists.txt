add_executable(PACEPico2
    Main.cpp

    psram.c
)

# some additional target-specific options
option(PICO2_EMULATOR_ON_CORE1 "Run emulator on core1" OFF)
option(PICO2_CPU_IN_RAM "Move main CPU interpreter+memory access code to RAM" OFF)

if(PICO2_EMULATOR_ON_CORE1)
    target_compile_definitions(PACEPico2 PRIVATE EMULATOR_ON_CORE1=1)
endif()

if(PICO2_CPU_IN_RAM)
    target_compile_definitions(PACEPico2 PRIVATE PICO_CPU_IN_RAM=1)
endif()

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

# driver selection based on boards
if(EXTRA_BOARD STREQUAL "stamp_carrier")
    set(DISPLAY_DRIVER "hstx")
endif()

# select HSTX anyway, but complain about it
if(NOT DISPLAY_DRIVER)
    set(DISPLAY_DRIVER "hstx")

    message("No DISPLAY_DRIVER set, assuming HSTX")
endif()

target_sources(PACEPico2 PRIVATE
    Display_${DISPLAY_DRIVER}.cpp
)

# faster flash access
target_compile_definitions(PACEPico2 PRIVATE PICO_EMBED_XIP_SETUP=1)
# for config.h
target_include_directories(PACEPico2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# usual setup
target_link_libraries(PACEPico2 PACECore PACEPicoShared PACEPicoBIOS hardware_dma pico_multicore pico_stdlib)

pico_enable_stdio_uart(PACEPico2 1)
pico_add_extra_outputs(PACEPico2)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PACEPico2.uf2
	DESTINATION bin
)