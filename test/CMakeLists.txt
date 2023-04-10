set(TEST_LIBS "")
set(TEST_TARGETS "")
set(TEST_FLAGS "")
include(cmakeut_add_cpp_test)

# --------------
find_package(Boost REQUIRED unit_test_framework timer system)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
set(TEST_LIBS "${TEST_LIBS};${Boost_UNIT_TEST_FRAMEWORK_LIBRARIES};${Boost_SYSTEM_LIBRARIES};${Boost_TIMER_LIBRARIES}")
set(TEST_FLAGS "${TEST_FLAGS};-Wno-c99-extensions")
# --------------

cmakeut_add_cpp_test(inclusion LIBS "${TEST_LIBS};${ariles2_pugixml_catkin_LIBRARIES}" FLAGS "${TEST_FLAGS}")
cmakeut_add_cpp_test(urdf LIBS "${TEST_LIBS};${ariles2_pugixml_catkin_LIBRARIES};${ariles2_yamlcpp_catkin_LIBRARIES}" FLAGS "${TEST_FLAGS}" WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
