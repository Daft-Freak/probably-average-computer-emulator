add_library(PACEPicoShared INTERFACE)

target_sources(PACEPicoShared INTERFACE
    fatfs/ff.c
    fatfs/ffunicode.c

    DiskIO.cpp
    Filesystem.cpp
    Storage.cpp
    USBHID.cpp
)

target_include_directories(PACEPicoShared INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPicoShared INTERFACE PACECore hardware_pio tinyusb_host)

pico_generate_pio_header(PACEPicoShared ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

#embed BIOS
set(BIOS_FILE bios-xt.rom)
set(BIOS_PATH ${CMAKE_CURRENT_LIST_DIR}/../)
set(BIOS_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
add_custom_command(
    OUTPUT bios.o
    WORKING_DIRECTORY ${BIOS_PATH}
    COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B armv6s-m --rename-section .data=.rodata,alloc,load,readonly,data,contents ${BIOS_FILE} ${CMAKE_CURRENT_BINARY_DIR}/bios.o
    DEPENDS ${BIOS_PATH}${BIOS_FILE}
)

add_library(PACEPicoBIOS bios.o)
target_include_directories(PACEPicoBIOS INTERFACE ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(PACEPicoBIOS PROPERTIES LINKER_LANGUAGE C)

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

    target_sources(PACEPicoBIOS PRIVATE disk-bios.o)
    target_compile_definitions(PACEPicoBIOS PUBLIC FIXED_DISK)
endif()