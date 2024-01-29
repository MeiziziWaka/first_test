#ifndef __USER_SERVER_H
#define __USER_SERVER_H

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* 服务器名称/密码 */
#define WIFI_PASSWORD       "66668888"
#define WIFI_SSID           "esp32-c3"

/* 服务器相关信息 */
#define SERVER_IP           "192.168.4.1"
#define SERVER_PORT         6666

/* 定义输入服务器套接字结构体 */
typedef struct InputSocket {

    char *iname;
    // uint8_t aid;
    uint8_t mac[6];
    int isocketnum;                     /* 服务器套接字 */
    char *isocketip;                    /* IP地址 */
    uint8_t ireceivebuf[100];           /* 接收缓冲区 */
    TaskHandle_t isocket_TaskHandler;   /* 任务句柄 */
    SemaphoreHandle_t isocket_Sem;      /* 信号量句柄 */
    struct InputSocket* pnext;
}InputSocket_t, *pInputSocket;

void userServer_Init(void);
void AddInputSocket(void);
pInputSocket GetInputSocket(char *name);

#endif /* __USER_SERVER_H */
