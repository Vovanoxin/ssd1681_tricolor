/*
 * Copyright (c) 2022 Andreas Sandberg
 * Copyright (c) 2018-2020 PHYTEC Messtechnik GmbH
 * Copyright 2024 NXP
 * Copyright 2025 Volodymyr Tesliuk <vovatesluk6a@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ssd1681_tricolor);

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/sys/byteorder.h>

#include <dt-bindings/display/ssd1681.h>
#include "ssd16xx_regs.h"

#define SSD16XX_PIXELS_PER_BYTE 8

struct ssd16xx_quirks {
    uint16_t max_width;
    uint16_t max_height;
    uint8_t pp_width_bits;
    uint8_t pp_height_bits;
    uint8_t ctrl2_full;
};

struct ssd16xx_data {
    enum display_orientation orientation;
    enum display_pixel_format pixel_format;
    size_t plane_bytes;
    uint8_t *black_plane;
    uint8_t *red_plane;
};


struct ssd16xx_config {
    const struct device *mipi_dev;
    const struct mipi_dbi_config dbi_config;
    struct gpio_dt_spec busy_gpio;
    const struct ssd16xx_quirks *quirks;
    uint16_t height;
    uint16_t width;
};


static inline void ssd16xx_busy_wait(const struct device *dev) {
    const struct ssd16xx_config *config = dev->config;
    int pin = gpio_pin_get_dt(&config->busy_gpio);

    while (pin > 0) {
        __ASSERT(pin >= 0, "Failed to get pin level");
        k_msleep(SSD16XX_BUSY_DELAY);
        pin = gpio_pin_get_dt(&config->busy_gpio);
    }
}

static inline int ssd16xx_write_cmd(const struct device *dev, uint8_t cmd, const uint8_t *data,
                                    size_t len) {
    const struct ssd16xx_config *config = dev->config;
    int err;

    ssd16xx_busy_wait(dev);

    err = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, cmd, data, len);
    mipi_dbi_release(config->mipi_dev, &config->dbi_config);
    return err;
}

static inline int ssd16xx_write_uint8(const struct device *dev, uint8_t cmd, uint8_t data) {
    return ssd16xx_write_cmd(dev, cmd, &data, 1);
}

static inline int ssd16xx_read_cmd(const struct device *dev, uint8_t cmd, uint8_t *data,
                                   size_t len) {
    const struct ssd16xx_config *config = dev->config;
    const struct ssd16xx_data *dev_data = dev->data;

    if (!dev_data->read_supported) {
        return -ENOTSUP;
    }

    ssd16xx_busy_wait(dev);

    return mipi_dbi_command_read(config->mipi_dev, &config->dbi_config, &cmd, 1, data, len);
}

static inline size_t push_x_param(const struct device *dev, uint8_t *data, uint16_t x) {
    const struct ssd16xx_config *config = dev->config;

    if (config->quirks->pp_width_bits == 8) {
        data[0] = (uint8_t)x;
        return 1;
    }

    if (config->quirks->pp_width_bits == 16) {
        sys_put_le16(sys_cpu_to_le16(x), data);
        return 2;
    }

    LOG_ERR("Unsupported pp_width_bits %u", config->quirks->pp_width_bits);
    return 0;
}

static inline size_t push_y_param(const struct device *dev, uint8_t *data, uint16_t y) {
    const struct ssd16xx_config *config = dev->config;

    if (config->quirks->pp_height_bits == 8) {
        data[0] = (uint8_t)y;
        return 1;
    }

    if (config->quirks->pp_height_bits == 16) {
        sys_put_le16(sys_cpu_to_le16(y), data);
        return 2;
    }

    LOG_ERR("Unsupported pp_height_bitsa %u", config->quirks->pp_height_bits);
    return 0;
}

static inline int ssd16xx_set_ram_param(const struct device *dev, uint16_t sx, uint16_t ex,
                                        uint16_t sy, uint16_t ey) {
    int err;
    uint8_t tmp[4];
    size_t len;

    len = push_x_param(dev, tmp, sx);
    len += push_x_param(dev, tmp + len, ex);
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, sy);
    len += push_y_param(dev, tmp + len, ey);
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CTRL, tmp, len);
    if (err < 0) {
        return err;
    }

    return 0;
}

static inline int ssd16xx_set_ram_ptr(const struct device *dev, uint16_t x, uint16_t y) {
    int err;
    uint8_t tmp[2];
    size_t len;

    len = push_x_param(dev, tmp, x);
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_RAM_XPOS_CNTR, tmp, len);
    if (err < 0) {
        return err;
    }

    len = push_y_param(dev, tmp, y);
    return ssd16xx_write_cmd(dev, SSD16XX_CMD_RAM_YPOS_CNTR, tmp, len);
}

static int ssd16xx_activate(const struct device *dev, uint8_t ctrl2) {
    int err;

    err = ssd16xx_write_uint8(dev, SSD16XX_CMD_UPDATE_CTRL2, ctrl2);
    if (err < 0) {
        return err;
    }

    return ssd16xx_write_cmd(dev, SSD16XX_CMD_MASTER_ACTIVATION, NULL, 0);
}

static int ssd16xx_update_display(const struct device *dev) {
    const struct ssd16xx_config *config = dev->config;
    const uint8_t update_cmd = SSD16XX_CTRL2_ENABLE_CLK | SSD16XX_CTRL2_ENABLE_ANALOG |
                               SSD16XX_CTRL2_LOAD_LUT | SSD16XX_CTRL2_LOAD_TEMPERATURE |
                               config->quirks->ctrl2_full |
                               SSD16XX_CTRL2_DISABLE_ANALOG | SSD16XX_CTRL2_DISABLE_CLK;
    return ssd16xx_activate(dev, update_cmd);
}

static int ssd16xx_blanking_off(const struct device *dev) {
    return ssd16xx_update_display(dev);
}

static int ssd16xx_blanking_on(const struct device *dev) {
    return 0;
}

static int ssd16xx_set_window(const struct device *dev, const uint16_t x, const uint16_t y,
                              const struct display_buffer_descriptor *desc) {
    const struct ssd16xx_config *config = dev->config;
    uint16_t x_start = 0;
    uint16_t x_end = (config->height - 1) / SSD16XX_PIXELS_PER_BYTE;
    uint16_t y_start = 0;
    uint16_t y_end = config->width - 1;
    int err;

    err = ssd16xx_set_ram_param(dev, x_start, x_end, y_start, y_end);
    if (err < 0) {
        return err;
    }

    return ssd16xx_set_ram_ptr(dev, x_start, y_start);
}


static int ssd16xx_write_tricolor(const struct device *dev, const uint16_t x, const uint16_t y,
                                  const struct display_buffer_descriptor *desc, const void *buf) {
    // Const colors for tricolor display
    const uint16_t PIX_RED = 0xF800;
    const uint16_t PIX_BLACK = 0x0000;

    const struct ssd16xx_data *data = dev->data;
    size_t bytes = data->plane_bytes;
    enum display_pixel_format pf = desc->pixel_format;
    const uint8_t *in = buf;

    int err;
    size_t plane_bytes = desc->height * desc->width / SSD16XX_PIXELS_PER_BYTE;

    memset(data->black_plane, 0, data->plane_bytes);
    memset(data->red_plane, 0, data->plane_bytes);

    /* build bit-planes using passed buffer*/
    for (uint16_t row = 0; row < desc->height; row++) {
        const uint8_t *row_ptr = in + row * desc->pitch;

        for (uint16_t col = 0; col < desc->width; col++) {
            /* load a big-endian 16-bit pixel */
            uint16_t pix = sys_get_be16(row_ptr + col * sizeof(uint16_t));

            bool black = (pix == PIX_BLACK);
            bool red = (pix == PIX_RED);

            /* one bit per pixel; compute byte index + mask */
            size_t bit = (size_t)row * desc->width + col;
            size_t idx = bit / SSD16XX_PIXELS_PER_BYTE;
            uint8_t mask = 1 << (SSD16XX_PIXELS_PER_BYTE - 1 - (bit % SSD16XX_PIXELS_PER_BYTE));

            if (black) {
                // Black: R=0, B/W=0 according to LUT table
                // Both planes already cleared to 0, so do nothing
            } else if (red) {
                // Red: R=1, B/W=0 according to LUT table
                data->red_plane[idx] |= mask;
            } else {
                // White: R=0, B/W=1 according to LUT table
                data->black_plane[idx] |= mask;
            }
        }
    }
    /* window + both planes + single activation */
    err = ssd16xx_set_window(dev, x, y, desc);
    if (err < 0) {
        return err;
    }

    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_WRITE_RAM, data->black_plane, bytes);
    if (err < 0) {
        return err;
    }

    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_WRITE_RED_RAM, data->red_plane, bytes);
    if (err < 0) {
        return err;
    }

    return ssd16xx_update_display(dev);
}


static int ssd16xx_write(const struct device *dev, const uint16_t x, const uint16_t y,
                         const struct display_buffer_descriptor *desc, const void *buf) {
    return ssd16xx_write_tricolor(dev, x, y, desc, buf);
}

static void ssd16xx_get_capabilities(const struct device *dev, struct display_capabilities *caps) {
    const struct ssd16xx_config *config = dev->config;
    struct ssd16xx_data *data = dev->data;

    memset(caps, 0, sizeof(struct display_capabilities));
    caps->x_resolution = config->width;
    caps->y_resolution = config->height;
    caps->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
    caps->current_pixel_format = PIXEL_FORMAT_RGB_565;
    caps->screen_info = SCREEN_INFO_MONO_MSB_FIRST | SCREEN_INFO_EPD;
    caps->current_orientation = data->orientation;
}

static int ssd16xx_set_pixel_format(const struct device *dev, const enum display_pixel_format pf) {
    struct ssd16xx_data *data = dev->data;
    if (pf == PIXEL_FORMAT_RGB_565) {
        data->pixel_format = PIXEL_FORMAT_RGB_565;
        return 0;
    }
    return -ENOTSUP;
}

static int ssd16xx_set_orientation(const struct device *dev,
                                   const enum display_orientation orientation) {
    struct ssd16xx_data *data = dev->data;
    uint8_t scan_mode = SSD16XX_DATA_ENTRY_XDYIY;
    int err;

    err = ssd16xx_write_uint8(dev, SSD16XX_CMD_ENTRY_MODE, scan_mode);
    if (err < 0) {
        return err;
    }

    data->orientation = orientation;
    return 0;
}




static int ssd16xx_controller_init(const struct device *dev) {
    const struct ssd16xx_config *config = dev->config;
    struct ssd16xx_data *data = dev->data;
    int err;
    uint8_t gdo[3];
    size_t gdo_len;

    data->plane_bytes = (config->width * config->height + SSD16XX_PIXELS_PER_BYTE - 1) / SSD16XX_PIXELS_PER_BYTE;
    data->black_plane = k_malloc(data->plane_bytes);
    data->red_plane = k_malloc(data->plane_bytes);
    if (!data->black_plane || !data->red_plane) {
        k_free(data->black_plane);
        k_free(data->red_plane);
        return -ENOMEM;
    }

    /* Hardware reset */
    err = mipi_dbi_reset(config->mipi_dev, SSD16XX_RESET_DELAY);
    if (err < 0) {
        return err;
    }
    k_msleep(2);

    /* Software reset */
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_SW_RESET, NULL, 0);
    if (err < 0) {
        return err;
    }
    k_msleep(1);

    /* Basic initialization */
    gdo_len = push_y_param(dev, gdo, config->width - 1);
    gdo[gdo_len++] = 0U;
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_GDO_CTRL, gdo, gdo_len);
    if (err < 0) {
        return err;
    }

    err = ssd16xx_set_orientation(dev, DISPLAY_ORIENTATION_NORMAL);
    if (err < 0) {
        return err;
    }

    /* Clear display to white */
    memset(data->black_plane, 0xFF, data->plane_bytes);
    memset(data->red_plane, 0x00, data->plane_bytes);
    
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_WRITE_RAM, data->black_plane, data->plane_bytes);
    if (err < 0) {
        return err;
    }
    
    err = ssd16xx_write_cmd(dev, SSD16XX_CMD_WRITE_RED_RAM, data->red_plane, data->plane_bytes);
    if (err < 0) {
        return err;
    }

    return ssd16xx_update_display(dev);
}

static int ssd16xx_init(const struct device *dev) {
    const struct ssd16xx_config *config = dev->config;
    int err;

    if (!device_is_ready(config->mipi_dev)) {
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&config->busy_gpio)) {
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->busy_gpio, GPIO_INPUT);
    if (err < 0) {
        return err;
    }

    return ssd16xx_controller_init(dev);
}

static DEVICE_API(display, ssd16xx_driver_api) = {
    .blanking_on = ssd16xx_blanking_on,
    .blanking_off = ssd16xx_blanking_off,
    .write = ssd16xx_write,
    .get_capabilities = ssd16xx_get_capabilities,
    .set_pixel_format = ssd16xx_set_pixel_format,
    .set_orientation = ssd16xx_set_orientation,
};

static struct ssd16xx_quirks quirks_solomon_ssd1681 = {
    .max_width = 200,
    .max_height = 200,
    .pp_width_bits = 8,
    .pp_height_bits = 16,
    .ctrl2_full = SSD16XX_GEN2_CTRL2_DISPLAY,
};


#define SSD16XX_DEFINE(n, quirks_ptr)                                                              \
    static const struct ssd16xx_config ssd16xx_cfg_##n = {                                         \
        .mipi_dev = DEVICE_DT_GET(DT_PARENT(n)),                                                   \
        .dbi_config =                                                                              \
            {                                                                                      \
                .mode = MIPI_DBI_MODE_SPI_4WIRE,                                                   \
                .config = MIPI_DBI_SPI_CONFIG_DT(                                                  \
                    n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_HOLD_ON_CS | SPI_LOCK_ON, 0),    \
            },                                                                                     \
        .busy_gpio = GPIO_DT_SPEC_GET(n, busy_gpios),                                              \
        .quirks = quirks_ptr,                                                                      \
        .height = DT_PROP(n, height),                                                              \
        .width = DT_PROP(n, width),                                                                \
    };                                                                                             \
                                                                                                   \
    static struct ssd16xx_data ssd16xx_data_##n;                                                   \
                                                                                                   \
    DEVICE_DT_DEFINE(n, ssd16xx_init, NULL, &ssd16xx_data_##n, &ssd16xx_cfg_##n, POST_KERNEL,      \
                     CONFIG_SSD1681_TRICOLOR_INIT_PRIORITY, &ssd16xx_driver_api)

DT_FOREACH_STATUS_OKAY_VARGS(solomon_ssd1681, SSD16XX_DEFINE, &quirks_solomon_ssd1681);
