/*
Copyright 2018 Engicoder
*/

#ifndef ATT56K_INT_H
#define ATT56K_INT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum att56k_click_mode_e
{
    ATT56K_CLICK_MODE_OFF,
    ATT56K_CLICK_MODE_MAKE,
    ATT56K_CLICK_MODE_MAKE_BREAK,
    ATT56K_CLICK_MODE_COUNT,
} att56k_click_mode_e;

void att56k_init(void);
bool att56k_has_data(void);
uint16_t att56k_recv(void);
void att56k_click_cycle(void);
bool att56k_device_detected(void);

#endif // ATT56K_INT_H
