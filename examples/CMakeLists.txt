cmake_minimum_required(VERSION 2.8)
project (AMC7812_EXAMPLES)

include(${CMAKE_SOURCE_DIR}/arduino-uno-crosscompile.cmake)
 
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
 
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")

### add example executable directories
add_executable(amc7812_test amc7812_test.c)
### add the dependant libraries
#target_link_libraries(amc7812_test amc7812)
