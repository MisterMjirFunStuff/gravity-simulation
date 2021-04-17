cmake_minimum_required(VERSION 3.10)

project(simulation)

add_executable(${PROJECT_NAME}
  src/main.c
)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY bin)

target_link_libraries(${PROJECT_NAME} jel SDL2 m)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

target_include_directories(${PROJECT_NAME} PUBLIC
  /usr/local/include
  /usr/include/SDL2
)

target_link_directories(${PROJECT_NAME} PUBLIC
  /usr/local/lib
)
