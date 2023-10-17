cmake_minimum_required(VERSION 3.22)

SET(VHACD_DIR v-hacd)
SET(VHACD_INCLUDE_DIR ${VHACD_DIR}/include)

SET(PROJECT_NAME v-hacd_lib)

file(GLOB SOURCES src/*.cpp)

# my personal motivation for this library, is to expose an interface to python, that I can use
project(${PROJECT_NAME}
  VERSION 0.1
  LANGUAGES CXX)

message(STATUS ${PROJECT_NAME})
message(STATUS ${VHACD_INCLUDE_DIR})
message(STATUS ${SOURCES})

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${VHACD_INCLUDE_DIR})

