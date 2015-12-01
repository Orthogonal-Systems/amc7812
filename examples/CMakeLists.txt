cmake_minimum_required(VERSION 2.8)
project (amc7812_examples)

include(${ARDUINO_SETTINGS_FILE})
 
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
 
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")

### add example executable directories
add_avr_executable(amc7812_test amc7812_test.c)
### add the dependant libraries
avr_target_link_libraries(amc7812_test AMC7812)
avr_target_link_libraries(amc7812_test USART)
