cmake_minimum_required (VERSION 3.5)

project(cobra)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_STANDARD 11)

include_directories("COBRA")
file(GLOB SOURCES "COBRA/*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/COBRA/main.cpp")

find_package( Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(cobra ${SOURCES})
target_link_libraries(cobra ${Boost_LIBRARIES})