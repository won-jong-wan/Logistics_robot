cmake_minimum_required(VERSION 2.8...3.13)
project(AstarForStm VERSION 0.1.0 LANGUAGES C)

include(CTest)
enable_testing()

set(PROJECT_NAME AstarForStm)
set(SRC lpp/linkedList.c staCell/box.c staCell/stack.c)

include_directories(lpp)
include_directories(dpp)
include_directories(staCell)

add_executable(${PROJECT_NAME} main.c ${SRC})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)