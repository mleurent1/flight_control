#include <stdint.h>
#include <stdbool.h>

#include "board.h"

volatile bool sensor_busy = false;
volatile bool osd_busy = false;
volatile bool sma_busy = false;

// Weak definitions of board functions that are not mandatory
__attribute__((weak)) void rf_write(uint8_t addr, uint8_t * data, uint8_t size) {}
__attribute__((weak)) void rf_read(uint8_t addr, uint8_t size) {}
__attribute__((weak)) void toggle_led2(bool en) {}
__attribute__((weak)) void osd_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size_out, uint8_t size_in) {}
__attribute__((weak)) void sma_transfer(uint8_t* data_out, uint8_t* data_in, uint8_t size_out, uint8_t size_in) {}
__attribute__((weak)) void runcam_send(uint8_t * data, uint8_t size) {}
__attribute__((weak)) void set_leds(uint8_t * grb) {}