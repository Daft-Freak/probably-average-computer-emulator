add_library(PACEPicoShared INTERFACE)

target_sources(PACEPicoShared INTERFACE USBHID.cpp)

target_include_directories(PACEPicoShared INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(PACEPicoShared INTERFACE PACECore tinyusb_host)