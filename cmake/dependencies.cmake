# Suppress CMake Dev Warning in SFML
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS
      1
      CACHE INTERNAL "No dev warnings")
endif()

include(FetchContent)
find_package(SFML 2.5.1 QUIET)
if(SFML_FOUND)
  message(STATUS "Found SFML")
else()
  # for linux on mathepool we need to fetch the compiled version
  if((UNIX AND NOT APPLE) AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    FetchContent_Declare(
      sfml
      URL https://www.sfml-dev.org/files/SFML-2.5.1-linux-gcc-64-bit.tar.gz)
  else()
    FetchContent_Declare(
      sfml
      GIT_REPOSITORY https://github.com/SFML/SFML.git
      GIT_TAG 2.5.1
      GIT_SHALLOW TRUE)
  endif()
  FetchContent_MakeAvailable(sfml)
endif()

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG main
  GIT_SHALLOW TRUE)

if(${CMAKE_VERSION} VERSION_LESS 3.14)
  macro(FetchContent_MakeAvailable NAME)
    FetchContent_GetProperties(${NAME})
    if(NOT ${NAME}_POPULATED)
      FetchContent_Populate(${NAME})
      add_subdirectory(${${NAME}_SOURCE_DIR} ${${NAME}_BINARY_DIR})
    endif()
  endmacro()
endif()

FetchContent_MakeAvailable(googletest)
