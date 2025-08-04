/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#ifndef _NRF_24_CONF_H_
#define _NRF_24_CONF_H_

#include "main.h"
#define hspiX hspi1
#define spi_w_timeout 1000
#define spi_r_timeout 1000
#define spi_rw_timeout 1000

#define csn_gpio_port GPIOB
#define csn_gpio_pin GPIO_PIN_4 // 3

#define ce_gpio_port GPIOB // A
#define ce_gpio_pin GPIO_PIN_5 // 12 // 4

// y te pongo el de interrupción en PA11

#endif

