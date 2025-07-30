/*
 * Copyright (c) 2022 Andreas Sandberg
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modifications (2025) by Volodymyr Tesliuk <vovatesluk6a@gmail.com>
 * 
 */

#ifndef ZEPHYR_INCLUDE_DISPLAY_SSD1681_TRICOLOR_H_
#define ZEPHYR_INCLUDE_DISPLAY_SSD1681_TRICOLOR_H_

#include <zephyr/drivers/display.h>

/**
 * SSD16xx RAM type for direct RAM access
 */
enum ssd1681_ram {
    /** The black RAM buffer. This is typically the buffer used to
     * compose the contents that will be displayed after the next
     * refresh.
     */
    SSD1681_RAM_BLACK = 0,
    /* The red RAM buffer. This is typically the old frame buffer
     * when performing partial refreshes or an additional color
     * channel.
     */
    SSD1681_RAM_RED,
};

#endif /* ZEPHYR_INCLUDE_DISPLAY_SSD1681_TRICOLOR_H_ */
