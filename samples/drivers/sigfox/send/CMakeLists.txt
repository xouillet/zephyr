# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sigfox_send)

if(CONFIG_CPU_CORTEX_M0PLUS)
zephyr_link_libraries(
	${CMAKE_CURRENT_SOURCE_DIR}/libs/SFX_LIB_MON_CM0+_GCC.a)
elseif(CONFIG_CPU_CORTEX_M3)
zephyr_link_libraries(
	${CMAKE_CURRENT_SOURCE_DIR}/libs/SFX_LIB_MON_CM3_GCC.a)
elseif(CONFIG_CPU_CORTEX_M4 OR CONFIG_CPU_CORTEX_M7)
zephyr_link_libraries(
	${CMAKE_CURRENT_SOURCE_DIR}/libs/SFX_LIB_MON_CM4_GCC.a)
else()
message(FATAL_ERROR "This project only works with ARM Cortex-M MCUs")
endif()

if(CONFIG_SIGFOX_ST_RETRIEVER)
if(CONFIG_CPU_CORTEX_M0PLUS)
zephyr_link_libraries(
	${CMAKE_CURRENT_SOURCE_DIR}/libs/ST_RETRIEVER_LIB_CM0+_GCC.a)
elseif(CONFIG_CPU_CORTEX_M3)
zephyr_link_libraries(
	${CMAKE_CURRENT_SOURCE_DIR}/libs/ST_RETRIEVER_LIB_CM3_GCC.a)
elseif(CONFIG_CPU_CORTEX_M4 OR CONFIG_CPU_CORTEX_M7)
zephyr_link_libraries(
	${CMAKE_CURRENT_SOURCE_DIR}/libs/ST_RETRIEVER_LIB_CM4_GCC.a)
endif()
endif()

zephyr_linker_sources_ifdef(CONFIG_SIGFOX RODATA iar.rtmodel.ld)
zephyr_linker_sources_ifdef(CONFIG_SIGFOX RODATA iar.vfe.ld)

zephyr_include_directories(inc)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
