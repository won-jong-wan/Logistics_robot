cmake_minimum_required(VERSION 3.0.0)
project(AstarForStm VERSION 0.1.0 LANGUAGES C)

include(CTest)
enable_testing()

set(PROJECT_NAME AstarForStm)
set(CELL cell/linkedList.c )

include_directories(cell)
include_directories(stack)

add_executable(${PROJECT_NAME} main.c ${CELL})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
