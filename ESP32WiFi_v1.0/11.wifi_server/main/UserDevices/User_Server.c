#include <string.h>
#include "User_Server.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "sys/socket.h"
#include "driver/uart.h"

/* The Declaration of this variables and functions. */
void Server_Task(void *);
void ServerRecv_Task(void *);

static uint8_t g_InputSockMac[4][6] = {

    {0x54, 0x32, 0x04, 0x47, 0xF7, 0x2C},
    {0x54, 0x32, 0x04, 0x47, 0xEA, 0x3C},
};

static InputSocket_t g_tInputSocket[4] = {

    {.isocketip = "192.168.4.2", .iname = "client01"},
    {.isocketip = "192.168.4.3", .iname = "client02"},
    {.isocketip = "192.168.4.4", .iname = "client03"},
    {.isocketip = "192.168.4.5", .iname = "client04"},
};

pInputSocket g_ptInputSockets;

/**
  * @brief  The function is used to Register an Input socket Event.
  * @retval None.
  */
/**
  * @brief	注册一个输入socket事件
  * @retval None.
  */
void InputSocketRegister(pInputSocket ptSocket)
{
    ptSocket->pnext = g_ptInputSockets;
    g_ptInputSockets = ptSocket;
}

/**
  * @brief  The function is used to Registering Multiple Input socket Events.
  * @retval None.
  */
/**
  * @brief	注册多个输入socket事件
  * @retval None.
  */
void AddInputSocket(void)
{
    for(int i = 0; i < sizeof(g_tInputSocket) / sizeof(g_tInputSocket[0]); i++) {

        InputSocketRegister(&g_tInputSocket[i]);
    }
}

/**
  * @brief  The function is used to Get the Input socket Event.
  * @retval pInputSocket.
  */
/**
  * @brief	获取输入socket事件
  * @retval pInputSocket.
  */
pInputSocket GetInputSocket(char *name)
{
    pInputSocket pSock = g_ptInputSockets;
    while(pSock) {

        if(strcmp(pSock->iname, name) == 0) {

            return pSock;
        }
        else pSock = pSock->pnext;
    }
    return NULL;
}

/**
  * @brief  The function is used to Register Semaphore handles for multiple input sockets.
  * @retval None.
  */
/**
  * @brief	注册多个输入socket的信号量句柄
  * @retval None.
  */
void AddInputSocket_SEM(void)
{
    for(int i = 0; i < sizeof(g_tInputSocket) / sizeof(g_tInputSocket[0]); i++) {

        g_tInputSocket[i].isocket_Sem = xSemaphoreCreateBinary();
    }
}

/**
  * @brief  The function is used to Register MAC addresses for multiple input sockets.
  * @retval None.
  */
/**
  * @brief	注册多个输入socket的MAC地址
  * @retval None.
  */
void AddInputSocket_MAC(void)
{
    for(int i = 0; i < sizeof(g_tInputSocket) / sizeof(g_tInputSocket[0]); i++) {
    
        memcpy(g_tInputSocket[i].mac, &g_InputSockMac[i][0], sizeof(g_tInputSocket[i].mac));
    }
}

/**
  * @brief  The function is used to Callback function for WIFI registration events.
  * @retval TaskHandle_t point.
  */
/**
  * @brief	WIFI注册事件的回调函数
  * @retval None.
  */
void Server_EventHandler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    /* STA接入事件 */
    if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STACONNECTED)) {

        ESP_LOGI("SERVER", "STA接入");
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
        ESP_LOGI("SERVER", "STA的aid: %d", event->aid);
        ESP_LOGI("SERVER", "STA的mac: "MACSTR"", MAC2STR(event->mac));
    }

    /* STA断开事件 */
    else if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_AP_STADISCONNECTED)) {

        ESP_LOGI("SERVER", "STA断开连接");
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI("SERVER", "STA的aid: %d", event->aid);
        ESP_LOGI("SERVER", "STA的mac: "MACSTR"", MAC2STR(event->mac));
        ESP_LOGI("SERVER", "断开原因: %d", event->reason);
        for(int i = 0; i < sizeof(g_tInputSocket) / sizeof(g_tInputSocket[0]); i++) {
            if(memcmp(g_tInputSocket[i].mac, event->mac, sizeof(g_tInputSocket[i].mac)) == 0) {

                shutdown(g_tInputSocket[i].isocketnum, SHUT_RDWR);
                close(g_tInputSocket[i].isocketnum);
            }
        }
    }

    /* STA分配IP事件，如果使用静态IP, STA将不会分配IP */
    else if((event_base == IP_EVENT) && (event_id == IP_EVENT_AP_STAIPASSIGNED)) {

        ESP_LOGI("SERVER", "STA分配到IP");
    }
}

/**
  * @brief  The function is used to initialize the server.
  * @retval TaskHandle_t point.
  */
/**
  * @brief	进行服务器初始化
  * @retval None.
  */
void userServer_Init(void)
{
    /* 初始化要 */

    /* 创建默认事件循环 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 注册所有的WIFI事件 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, Server_EventHandler, NULL, NULL));

    /* 注册STA已分配IP事件 */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, Server_EventHandler, NULL, NULL));

    /* 初始化网卡的底层配置 */
    ESP_ERROR_CHECK(esp_netif_init());
    
    /* 创建AP网卡 */
    esp_netif_create_default_wifi_ap();

    /* WiFi初始化配置 */
    wifi_init_config_t Init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&Init_config));

    /* 设置WiFi模式 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    /* 设置AP属性 */
    wifi_config_t wifi_config = {

        .ap = {

            .authmode           = WIFI_AUTH_WPA_WPA2_PSK,  /* 认证方式 */
            .beacon_interval    = 100,                     /* 信标帧间隔 */
            .channel            = 1,                       /* 信道 */
            .ftm_responder      = false,                   /* WiFi测距，暂不需要 */
            .max_connection     = 4,                       /* 允许连接的最大站点数 */
            .pairwise_cipher    = WIFI_CIPHER_TYPE_NONE,   /* 加密方式 */
            .password           = WIFI_PASSWORD,           /* AP密码 */
            .ssid               = WIFI_SSID,               /* AP名称 */
            .ssid_hidden        = 0,                       /* 广播SSID */
            .ssid_len           = strlen(WIFI_SSID),
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    /* 启动WiFi */
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI("SERVER", "WiFi初始化配置完成开始创建服务器任务");

    AddInputSocket_SEM();
    AddInputSocket_MAC();

    /* 创建一个任务，用于配置服务器 */
    xTaskCreate((TaskFunction_t)Server_Task,
                "server",
                1024*2,
                NULL,
                1,
                NULL);

    xTaskCreate((TaskFunction_t)ServerRecv_Task,
                "server-recv01",
                1024*2,
                "client01",
                2,
                &g_tInputSocket[0].isocket_TaskHandler);

    xTaskCreate((TaskFunction_t)ServerRecv_Task,
                "server-recv02",
                1024*2,
                "client02",
                2,
                &g_tInputSocket[1].isocket_TaskHandler);
}

/**
  * @brief  Server task function.
  * @retval None.
  */
/**
  * @brief	服务器任务函数
  * @retval None.
  */
void Server_Task(void* pvParameter)
{
    TickType_t xTicksToWait = pdMS_TO_TICKS(1000ul);
    /* 创建服务器套接字 */
    int socketnum = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if(socketnum < 0) {
        
        ESP_LOGI("SERVER", "创建套接字失败");
        vTaskDelete(NULL);
    }

    /* 绑定服务器套接字信息 */
    struct sockaddr_in sockaddr = {

        .sin_addr.s_addr = inet_addr(SERVER_IP),
        .sin_family      = AF_INET,
        .sin_len         = sizeof(sockaddr),
        .sin_port        = htons(SERVER_PORT),
    };
    int err = bind(socketnum, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if(err < 0) {

        ESP_LOGI("SERVER", "绑定服务器套接字失败");
        vTaskDelete(NULL);
    }

    /* 开启服务器套接字的监听 */
    err = listen(socketnum, 1);
    if(err < 0) {

        ESP_LOGI("SERVER", "监听失败");
        vTaskDelete(NULL);
    }

    /* listen后，服务器会自动监听、检测客户端发来的连接请求 */
    while(1) {

        /* 接收客户端连接; accept函数是阻塞的, 一次只能创建一个连接 */
        struct sockaddr_in temp_socketaddr;
        socklen_t temp_socklen = sizeof(temp_socketaddr);
        int temp_socket =  accept(socketnum, (struct sockaddr*)&temp_socketaddr, &temp_socklen);
        if(temp_socket < 0) ESP_LOGI("SERVER", "接收客户端连接失败");
        ESP_LOGI("SERVER","分配服务器套接字 %d", temp_socket);

        for(int i = 0; i < 4; i++) {

            /* 检查客户端连接IP，分配服务器套接字并唤醒指定客户端任务 */
            if(strcmp(g_tInputSocket[i].isocketip, inet_ntoa(temp_socketaddr.sin_addr.s_addr)) == 0) {

                g_tInputSocket[i].isocketnum = temp_socket;
                xTaskNotifyGive(g_tInputSocket[i].isocket_TaskHandler);
                break;
            }
        }

        vTaskDelay(xTicksToWait);
    }
}

/**
  * @brief  Server receive task function.
  * @retval None.
  */
/**
  * @brief	服务器接收任务函数
  * @retval None.
  */
void ServerRecv_Task(void *pvParameter)
{
    pInputSocket psock = GetInputSocket((char*)pvParameter);
    TickType_t xTicksToWait = pdMS_TO_TICKS(350ul);
    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("SERVER", "%s被唤醒", (char*)pvParameter);

        while(1) {
    
            if(xSemaphoreTake(psock->isocket_Sem, portMAX_DELAY) == pdTRUE) {

                vTaskDelay(xTicksToWait);
                int len = recv(psock->isocketnum, psock->ireceivebuf, sizeof(psock->ireceivebuf), MSG_DONTWAIT);
                ESP_LOGI("SERVER", "len = %d", len);
                if(len <= 0) {
                    ESP_LOGI("SERVER", "服务器下线");
                    // break;
                }
                else {
                    uart_write_bytes(UART_NUM_1, psock->ireceivebuf, len);
                    memset(psock->ireceivebuf, 0, len);
                }
            }
        }
    }
}
