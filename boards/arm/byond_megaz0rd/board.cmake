# SPDX-License-Identifier: Apache-2.0

# Enables SAM-BA bootloader on Zephyr west
include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)

board_runner_args(jlink "--device=atsam4e16c" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
