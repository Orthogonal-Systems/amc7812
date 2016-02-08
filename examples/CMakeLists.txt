cmake_minimum_required(VERSION 2.8)
project (amc7812_examples)

include(${ARDUINO_SETTINGS_FILE})
 
set(PROJECT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
 
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_INCLUDE_DIR}")

### add example executable directories
add_avr_executable(amc7812_test amc7812_test/amc7812_test.cpp)
add_avr_executable(amc7812_stream amc7812_stream/amc7812_stream.cpp)
add_avr_executable(amc7812_trig-daq amc7812_trig-daq/amc7812_trig-daq.cpp)

### add the dependant libraries
### acm7812_test
avr_target_link_libraries(amc7812_test AMC7812)
avr_target_link_libraries(amc7812_test ARDUINO)
### acm7812_stream
avr_target_link_libraries(amc7812_stream AMC7812)
avr_target_link_libraries(amc7812_stream ARDUINO)
avr_target_link_libraries(amc7812_stream UIPETHERNET)
### acm7812_trig-daq
avr_target_link_libraries(amc7812_trig-daq AMC7812)
avr_target_link_libraries(amc7812_trig-daq ARDUINO)
avr_target_link_libraries(amc7812_trig-daq UIPETHERNET)
