cmake_minimum_required(VERSION 3.14)
project(
  dirsig5b
    VERSION 0.0.1
    DESCRIPTION "dirsig5b"
    LANGUAGES CXX
    )
option(BUILD_NATIVE "Build and tune for native hardware?" ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(CheckCXXCompilerFlag)
macro(add_cxx_flag_if_available WHICH_FLAGS FLAG)
  set(FLAG_OK)
  check_cxx_compiler_flag("${FLAG}" FLAG_OK)
  if(FLAG_OK)
    string(APPEND ${WHICH_FLAGS} " ${FLAG}")
  endif()
endmacro()
add_cxx_flag_if_available(CMAKE_CXX_FLAGS "-Wall")
add_cxx_flag_if_available(CMAKE_CXX_FLAGS "-Wextra")
add_cxx_flag_if_available(CMAKE_CXX_FLAGS_DEBUG "-fno-omit-frame-pointer")
add_cxx_flag_if_available(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=address")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "CXX Flags Debug: ${CMAKE_CXX_FLAGS_DEBUG}")

find_package(Microcosm REQUIRED)
find_package(OpenMP REQUIRED)
add_subdirectory(lib)
add_subdirectory(src)

