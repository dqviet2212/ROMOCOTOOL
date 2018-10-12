cmake_minimum_required(VERSION 2.6)

project(cmake_test)

set(PROJECT_INCLUDE_DIR include)
set(PROJECT_SRC src/A.cpp src/B.cpp src/C.cpp src/main.cpp)
set(PROJECT_TARGET ${PROJECT_NAME}_main)

include_directories(${PROJECT_INCLUDE_DIR})
add_executable(${PROJECT_TARGET} ${PROJECT_SRC})

# add the install targets
install(TARGETS ${PROJECT_TARGET} DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

