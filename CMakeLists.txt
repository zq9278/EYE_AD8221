#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(EYE_AD8221)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/emg_notch.c
  src/emg_sampler.c
  src/emg_uart_stream.c
  src/emg_ble_stream.c
)

# NORDIC SDK APP END
