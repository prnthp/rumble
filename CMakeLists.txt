# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/nau8325
    )

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(audio)

set(SOURCES
    src/main.c
    )

target_sources(app PRIVATE ${SOURCES})
