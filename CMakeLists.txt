# General parameters
cmake_minimum_required(VERSION 3.0)
project(ariles_urdf VERSION 0.1.0 LANGUAGES CXX)


# --------------
include(CMakeParseArguments)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(cmakeut_compiler_flags)
# --------------


# --------------
# Depedencies
# --------------
find_package(ariles2_yamlcpp_catkin REQUIRED)
find_package(ariles2_pugixml_catkin REQUIRED)
find_package(cpput REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories (SYSTEM "${EIGEN3_INCLUDE_DIR}")
# --------------


# --------------
# Flags
# --------------
cmakeut_compiler_flags("c++11")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKEUT_CXX_FLAGS}")
#if (CCWS_CLANG_TIDY)
#    set (CMAKE_CXX_CLANG_TIDY "${CCWS_CLANG_TIDY}")
#endif()
#if (CCWS_CXX_FLAGS)
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CCWS_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CCWS_LINKER_FLAGS}")
#endif()
#add_definitions(-DCCWS_DEBUG=${CCWS_DEBUG})
#add_definitions(-DCCWS_BUILD_PROFILE="${CCWS_BUILD_PROFILE}")
# --------------


include_directories ("${PROJECT_SOURCE_DIR}/include")

enable_testing()
add_subdirectory("${PROJECT_SOURCE_DIR}/test")


install (DIRECTORY "${PROJECT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/include/${CMAKE_PROJECT_NAME}/"
         FILES_MATCHING PATTERN "*.h")
