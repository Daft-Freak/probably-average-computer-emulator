add_executable(PACETest Main.cpp)

find_package(ZLIB REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

target_link_libraries(PACETest PRIVATE PACECore nlohmann_json::nlohmann_json ZLIB::ZLIB)