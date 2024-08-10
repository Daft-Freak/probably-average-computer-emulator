# embed BIOS
set(BIOS_FILE bios-xt.rom)
set(BIOS_PATH ${CMAKE_CURRENT_LIST_DIR}/../)
add_custom_command(
    OUTPUT bios.o
    WORKING_DIRECTORY ${BIOS_PATH}
    COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B armv6s-m --rename-section .data=.rodata,alloc,load,readonly,data,contents ${BIOS_FILE} ${CMAKE_CURRENT_BINARY_DIR}/bios.o
    DEPENDS ${BIOS_PATH}${BIOS_FILE}
)

add_executable(PACEPicoPlus2 bios.o
    dvhstx/dvhstx.cpp
    dvhstx/dvi.cpp

    Main.cpp
)

# embed disk adapter BIOS if found
set(DISK_BIOS_FILE fixed-disk-bios.rom)
if(EXISTS ${BIOS_PATH}/${DISK_BIOS_FILE})
    add_custom_command(
        OUTPUT disk-bios.o
        WORKING_DIRECTORY ${BIOS_PATH}
        COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B armv6s-m --rename-section .data=.rodata,alloc,load,readonly,data,contents ${DISK_BIOS_FILE} ${CMAKE_CURRENT_BINARY_DIR}/disk-bios.o
        DEPENDS ${BIOS_PATH}${DISK_BIOS_FILE}
    )

    message("Found fixed disk adapter BIOS")

    target_sources(PACEPicoPlus2 PRIVATE disk-bios.o)
    target_compile_definitions(PACEPicoPlus2 PRIVATE FIXED_DISK)
endif()

target_include_directories(PACEPicoPlus2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPicoPlus2 PACECore PACEPicoShared hardware_dma pico_stdlib)

pico_enable_stdio_uart(PACEPicoPlus2 1)
pico_add_extra_outputs(PACEPicoPlus2)