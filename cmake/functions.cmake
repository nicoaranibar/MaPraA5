# Copyright (c) 2022, The MaPra Authors.

function(mapra_set_standard_target_options OBJECTNAME)
  set_target_properties(
    ${OBJECTNAME} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                             CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON)
  target_compile_options(
    ${OBJECTNAME}
    PRIVATE
      $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-Wall
      -Wextra
      -Wpedantic
      -fno-inline
      -fPIC
      -fPIE>)
  target_include_directories(${OBJECTNAME}
                             PRIVATE "${mapra_SOURCE_DIR}/include/")
endfunction()

function(mapra_add_test TESTNAME)
  add_executable(${TESTNAME} "tests/${TESTNAME}.cpp" ${ARGN} "tests/run.cpp")
  target_link_libraries(${TESTNAME} PRIVATE gtest gmock)
  mapra_set_standard_target_options(${TESTNAME})
  add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
  set_target_properties(${TESTNAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                               "${CMAKE_BINARY_DIR}/tests")
endfunction()

function(mapra_add_library NAME)
  add_library(${NAME} ${ARGN})
  mapra_set_standard_target_options(${NAME})
  target_include_directories(${NAME} PUBLIC "/opt/X11/include")
  target_link_libraries(${NAME} PRIVATE GL X11 Threads::Threads)
  target_link_directories(${NAME} PRIVATE "/opt/X11/lib")
endfunction()

function(mapra_add_executable NAME)
  add_executable(${NAME} ${ARGN})
  mapra_set_standard_target_options(${NAME})
  target_include_directories(${NAME} PUBLIC "/opt/X11/include")
  target_link_libraries(
    ${NAME}
    PRIVATE
      GL
      X11
      Threads::Threads
      $<$<CXX_COMPILER_ID:AppleClang>:${mapra_SOURCE_DIR}/lib/libMapraUnitMac.a>
      $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:${mapra_SOURCE_DIR}/lib/libMapraUnit.a>
  )
  set_target_properties(${NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                           "${CMAKE_BINARY_DIR}/bin")
  target_link_directories(${NAME} PRIVATE "/opt/X11/lib")
endfunction()
