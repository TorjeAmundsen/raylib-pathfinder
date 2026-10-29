cmake_minimum_required(VERSION 3.24)
project(raylib-pathfinder C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG         6.2
    GIT_SHALLOW     TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(raylib-pathfinder src/main.c)
target_link_libraries(raylib-pathfinder PRIVATE raylib)

if(MSVC)
    target_compile_options(raylib-pathfinder PRIVATE /W4)
endif()
