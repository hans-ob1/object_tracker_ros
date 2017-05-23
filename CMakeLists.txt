cmake_minimum_required(VERSION 2.8.12)

add_subdirectory(dlib)

ADD_EXECUTABLE(hungarian src/max_cost_assignment_ex.cpp)
TARGET_LINK_LIBRARIES(hungarian dlib::dlib )
