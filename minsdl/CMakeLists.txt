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