cmake_minimum_required(VERSION 3.0.0)
project(pp VERSION 0.1.0 LANGUAGES C)

include(CTest)
enable_testing()

set(HEAD main.h)
set(SRC main.c)

add_executable(pp ${HEAD} ${SRC})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
