cmake_minimum_required(VERSION 2.8.3)
project(RP_project)
include_directories(. /usr/include/eigen3)
 
add_executable(static_vec_test
  static_vec_test.cpp)

add_executable(static_mat_test
  static_mat_test.cpp)

add_executable(my_ik
  my_ik.cpp)
  


