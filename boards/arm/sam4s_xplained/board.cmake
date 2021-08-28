# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--iface=JTAG")
board_runner_args(jlink "--speed=4000")
board_runner_args(jlink "--tool-opt=-jtagconf -1,-1")
board_runner_args(jlink "--tool-opt=-autoconnect 1")
board_runner_args(jlink "--device=atsam4s16c")
board_runner_args(jlink "--reset-after-load")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)
