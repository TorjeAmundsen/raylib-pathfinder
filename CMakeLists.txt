cmake_minimum_required(VERSION 3.26)
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

if(NOT WIN32)
    target_link_libraries(raylib-pathfinder PRIVATE m)
endif()

if(MSVC)
    target_compile_options(raylib-pathfinder PRIVATE /W4)
    target_link_options(raylib-pathfinder PRIVATE
        $<$<CONFIG:Release>:/SUBSYSTEM:WINDOWS>
        $<$<CONFIG:Release>:/ENTRY:mainCRTStartup>
    )
else()
    target_compile_options(raylib-pathfinder PRIVATE -Wall -Wextra)
endif()

if(MINGW)
    target_link_options(raylib-pathfinder PRIVATE
        -static
        $<$<CONFIG:Release>:-mwindows>
    )
endif()
