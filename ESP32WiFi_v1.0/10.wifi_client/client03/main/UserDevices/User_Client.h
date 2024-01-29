#ifndef __USER_CLIENT_H
#define __USER_CLIENT_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOGTEMP             "Clinet"
#define ESP_WIFI_SSID       "esp32-c3"
#define ESP_WIFI_PASS       "66668888"
#define WIFI_STA_IP         "192.168.4.4"
#define WIFI_STA_NETMASK    "255.255.255.0"
#define WIFI_STA_GW         "192.168.4.1"

typedef enum {

    CONNECT_INIT    = 0,
    CONNECT_SUCCESS = 1,
    CONNECT_OFFLINE_NORMAL = 2,
    CONNECT_OFFLINE_ERROR  = 3,
}SERVER_CONNECT_STATE_E;

extern TaskHandle_t g_xClientTaskHandle;

void Client_Init(void);
int Client_GetSocknum(void);
void Client_Task(void *pvParameter);

#endif /* __USER_CLIENT_H */
