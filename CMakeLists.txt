# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SSD1681_TRICOLOR)
  zephyr_library()
  zephyr_library_sources(drivers/display/ssd1681_tricolor.c)
  zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()