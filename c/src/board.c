#include <stdint.h>
#include "board.h"

// Weak definitions of board functions that are not mandatory
void __attribute__((weak)) rf_write(uint8_t addr, uint8_t * data, uint8_t size) {}
void __attribute__((weak)) rf_read(uint8_t addr, uint8_t size) {}
void __attribute__((weak)) toggle_led2(_Bool en) {}
void __attribute__((weak)) osd_send(uint8_t * data, uint8_t size) {}
void __attribute__((weak)) runcam_send(uint8_t * data, uint8_t size) {}
void __attribute__((weak)) sma_send(uint8_t * data, uint8_t size) {}
void __attribute__((weak)) set_leds(uint8_t * grb) {}