cmake_minimum_required(VERSION 2.8.12)
project(nbody)

add_definitions("-std=c++17")

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()


file(GLOB NBodySRC
    "src/header/*.hpp"
    "src/*.cpp"
)

add_executable(${PROJECT_NAME} ${NBodySRC})
target_link_libraries(${PROJECT_NAME} ${CONAN_LIBS})
