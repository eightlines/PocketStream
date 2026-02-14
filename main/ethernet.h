#ifndef ETHERNET_H
#define ETHERNET_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define GOT_IP_BIT BIT0

extern EventGroupHandle_t s_eth_event_group;

void init_ethernet(void);

#endif // ETHERNET_H
