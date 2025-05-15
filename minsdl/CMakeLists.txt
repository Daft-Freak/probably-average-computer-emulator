# minimal SDL shell

add_executable(PACE_SDL
    DiskIO.cpp
    Main.cpp
)

find_package(SDL2 REQUIRED)

target_link_libraries(PACE_SDL PACECore SDL2::SDL2)

if(SDL2_SDL2main_FOUND)
    target_link_libraries(PACE_SDL SDL2::SDL2main)
endif()

install(TARGETS PACE_SDL)

# install SDL2.dll on windows for convenience
if(WIN32)
    get_target_property(SDL2_LOCATION SDL2::SDL2 IMPORTED_LOCATION_RELEASE)
    if(NOT SDL2_LOCATION)
        get_target_property(SDL2_LOCATION SDL2::SDL2 IMPORTED_LOCATION)
    endif()

    if(SDL2_LOCATION MATCHES ".dll$")
        install(FILES ${SDL2_LOCATION} DESTINATION bin)
    endif()
endif()