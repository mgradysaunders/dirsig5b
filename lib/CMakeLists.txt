add_library(
  dirsig5b 
  SHARED
  LocalSurface.cc
  LocalVolume.cc
  Scattering.cc
  Simulation.cc
  World.cc
  )
target_include_directories(
  dirsig5b 
  PUBLIC
    $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${${PROJECT_NAME}_BINARY_DIR}/include>
  )
set_target_properties(
  dirsig5b
  PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME dirsig5b
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  )
target_link_libraries(
  dirsig5b
  PUBLIC
    Microcosm::Geometry
    Microcosm::Render
    OpenMP::OpenMP_CXX
    ${CMAKE_DL_LIBS}
  )
if(BUILD_NATIVE AND UNIX) 
  target_compile_options(
    dirsig5b PRIVATE "$<$<CONFIG:RELEASE>:-O3;-DNDEBUG;-march=native;-mtune=native>")
endif()
include(GenerateExportHeader)
generate_export_header(
  dirsig5b
  EXPORT_MACRO_NAME D5B_API
  EXPORT_FILE_NAME "${PROJECT_BINARY_DIR}/include/dirsig5b/Export.h"
  )
