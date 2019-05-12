###
### CMakeプロジェクト設定
###
# CMakeのバージョンを設定
cmake_minimum_required(VERSION 2.8)
# プロジェクト名と使用する言語を設定
project(dynamics_simulator CXX)

###
### option
###
# option(rls_thirdparty_cddlib "link Double description library" OFF)
#set(USE_FEATURE_X ${PROJECT_SOURCE_DIR}/share/ CACHE STRING "The directory containing")

# build dir make and set bin dir
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}//include)

###
### サブディレクトリを登録
###
if(rls_thirdparty_cddlib)
  #include_directories(thirdparty)
  add_subdirectory(thirdparty)
endif()

include_directories(include)
include_directories(matplotlib-cpp)

add_subdirectory(src)

# configure_file(
#   "${CMAKE_CURRENT_SOURCE_DIR}/include/friction.hpp.in"
#   "${CMAKE_CURRENT_SOURCE_DIR}/include/friction.hpp"
#   )

###
### Message
###
# message (STATUS "rls_thirdparty_cddlib=${rls_thirdparty_cddlib}")

