cmake_minimum_required(VERSION 3.0)
project(rhoban_utils)

find_package (Eigen3 3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)

option(RHOBAN_UTILS_BUILD_EXAMPLES "Building examples" OFF)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

#Enable Weird option linker
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

#List Sources sub source directories
set(SOURCES_DIRECTORIES
  src/rhoban_utils
  src/rhoban_utils/control
  src/rhoban_utils/history
  src/rhoban_utils/logging
  src/rhoban_utils/nominal
  src/rhoban_utils/serialization
  src/rhoban_utils/sockets
  src/rhoban_utils/spline
  src/rhoban_utils/tables
  src/rhoban_utils/threading
  src/rhoban_utils/timing
)

set (EXTRA_LIBRARIES
    Eigen3::Eigen
    jsoncpp_lib
)

#Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
    message(STATUS "-- Sources: Adding ${DIRECTORY}/")
    include (${DIRECTORY}/Sources.cmake)
    set (PREFIXED_SOURCES)
    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)
    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

add_library (rhoban_utils ${ALL_SOURCES})
target_include_directories(rhoban_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rhoban_utils ${EXTRA_LIBRARIES} Threads::Threads)

if (RHOBAN_UTILS_BUILD_EXAMPLES)
  add_executable(json_serialization_example examples/json_serialization.cpp)
  target_link_libraries(json_serialization_example rhoban_utils ${EXTRA_LIBRARIES} Threads::Threads)
  add_executable(json_factory_example examples/json_factory.cpp)
  target_link_libraries(json_factory_example rhoban_utils ${EXTRA_LIBRARIES} Threads::Threads)
endif()

 
set(TESTS
  angle
  control/bang_bang
  control/control
  control/pid
  control/variation_bound
  history/history
  logging/csv
  logging/logger
  logging/term_display
  # nominal/nominal not tested -> considered experimental
  serialization/stream_serializable
  serialization/json_serializable
  spline/function
  spline/poly_spline
  tables/double_table
  tables/string_table
  # threading/condition not tested -> adapter for condition API
  threading/multi_core
  # threading/mutex not tested -> adapter for mutex API
  # threading/thread not tested -> adapter for thread API
#  timing/benchmark # TO be fixed: private variables
  timing/time_stamp
  timing/elapse_tick
  timing/sleep
  util
)

include(CTest)
if (BUILD_TESTING)
  if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/tests)
  endif()

  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  # create an executable for each test
  foreach(test ${TESTS})
    string(REPLACE "/" "_" id ${test})
    add_executable(${id} ${CMAKE_CURRENT_SOURCE_DIR}/tests/${test}.cpp)
    add_test(NAME ${id} COMMAND ${id})
    target_link_libraries(${id} PRIVATE rhoban_utils GTest::GTest)
    set_target_properties(${id} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tests"
    )
  endforeach(test)
endif()
