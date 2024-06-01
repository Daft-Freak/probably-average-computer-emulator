include(picovision_import.cmake)

# embed BIOS
set(BIOS_FILE bios-xt.rom)
set(BIOS_PATH ${CMAKE_CURRENT_LIST_DIR}/../)
add_custom_command(
    OUTPUT bios.o
    WORKING_DIRECTORY ${BIOS_PATH}
    COMMAND ${CMAKE_OBJCOPY} -I binary -O elf32-littlearm -B armv6s-m --rename-section .data=.rodata,alloc,load,readonly,data,contents ${BIOS_FILE} ${CMAKE_CURRENT_BINARY_DIR}/bios.o
    DEPENDS ${BIOS_PATH}${BIOS_FILE}
)

add_executable(PACEPicoVision bios.o Display.cpp Main.cpp)

target_include_directories(PACEPicoVision PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPicoVision PACECore hardware_i2c pico_stdlib tinyusb_host aps6404 swd_load)

pico_add_extra_outputs(PACEPicoVision)