# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_SIGFOX_S2LP st_s2lp.c)
zephyr_sources_ifdef(CONFIG_SIGFOX_S2LP st_s2lp_rf.c)
zephyr_sources_ifdef(CONFIG_SIGFOX_S2LP st_s2lp_iface.c)
