# Copyright (c) 2024 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ZMK_BEHAVIOR_DRV883X src/behavior_drv883x.c)
zephyr_library_sources_ifdef(CONFIG_DRV883X src/drv883x.c)
zephyr_include_directories(include)

zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
