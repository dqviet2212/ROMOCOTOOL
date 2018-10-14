cmake_minimum_required(VERSION 2.6)

project(romocotool)

set(PROJECT_INCLUDE_DIR include)
set(PROJECT_SRC src/OCP.cpp src/NLP.cpp src/IPOPT.cpp src/optTraj.cpp)
set(PROJECT_TARGET ${PROJECT_NAME}_optTraj)

include_directories(${PROJECT_INCLUDE_DIR})
add_executable(${PROJECT_TARGET} ${PROJECT_SRC})

# add the install targets
install(TARGETS ${PROJECT_TARGET} DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

