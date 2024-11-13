cmake_minimum_required(VERSION 3.13.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

if(PICO_SDK_PATH OR PICO_SDK_FETCH_FROM_GIT)
  include(pico_sdk_import.cmake)
endif()

project(ProbablyAverageComputorEmulator)

if(PICO_SDK_PATH) # set by import file
  set(IS_PICO true)
  pico_sdk_init()
endif()

if(MSVC)
  add_compile_options("/W4" "/wd4244" "/wd4324" "/wd4458" "/wd4100")
else()
  add_compile_options("-Wall" "-Wextra" "-Wdouble-promotion" "-Wno-unused-parameter")
endif()

include(CMakeDependentOption)
cmake_dependent_option(BUILD_SDL "Build minimal SDL UI" ON "NOT IS_PICO" OFF)
cmake_dependent_option(BUILD_PICOVISION "Build PicoVision UI" ON "IS_PICO" OFF)

add_subdirectory(core)

if(IS_PICO)
  add_subdirectory(pico-shared)
endif()

if(BUILD_SDL)
  add_subdirectory(minsdl)
endif()

if(BUILD_PICOVISION)
  add_subdirectory(picovision)
endif()

# setup release packages
set(PROJECT_DISTRIBS LICENSE README.md)
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)
