cmake_minimum_required(VERSION 3.5)
  
project(SerialTest)

find_package(CppLinuxSerial REQUIRED)

add_executable(SerialTestExe test.cpp)
 
target_link_libraries(SerialTestExe CppLinuxSerial)
