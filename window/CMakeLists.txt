cmake_minimum_required(VERSION 3.5.0)
project(pp VERSION 0.1.0 LANGUAGES C)

set(HEAD main.h)
set(SRC main.c)

add_executable(pp ${HEAD} ${SRC})
