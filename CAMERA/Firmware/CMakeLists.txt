cmake_minimum_required(VERSION 3.28)
project(Firmware C)

set(CMAKE_C_STANDARD 11)

add_executable(Firmware main.c
        i2c_tools.c
        i2c_tools.h
        pwm_gen.c
        pwm_gen.h)
