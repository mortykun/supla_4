# !!! WARNING !!! AUTO-GENERATED FILE, PLEASE DO NOT MODIFY IT AND USE
# https://docs.platformio.org/page/projectconf/section_env_build.html#build-flags
#
# If you need to override existing CMake configuration or add extra,
# please create `CMakeListsUser.txt` in the root of project.
# The `CMakeListsUser.txt` will not be overwritten by PlatformIO.

cmake_minimum_required(VERSION 3.13)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

project("supla_4" C CXX)

include(CMakeListsPrivate.txt)

include_directories("${CMAKE_CURRENT_LIST_DIR}/lib/WifiSetup")

add_executable(SUPLA_BUILD ${SRC_LIST} lib/WifiSetup/WifiSetup.cpp lib/WifiSetup/WifiSetup.h)
