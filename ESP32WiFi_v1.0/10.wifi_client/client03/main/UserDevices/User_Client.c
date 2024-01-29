#include "User_Client.h"
#include "User_Uart.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "sys/socket.h"
#include "TMCx300.h"

/* The declaration of this variables and functions. */
int g_sockport;                     /* 服务器的套接字 */
TaskHandle_t g_xClientTaskHandle;   /* 客户端任务句柄 */
uint8_t g_receivebuff[100];         /* 客户端接收缓冲区 */

void Client_Task(void *pvParameter);

/* 注册事件的回调函数 */
void Client_EventHandler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    /* 配置好sta网卡以及wifi后，调用函数esp_wifi_start()后触发 */
    if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_START)) {

        ESP_LOGI(LOGTEMP, "开始连接");
        esp_wifi_connect();
    }

    /* 连接失败，尝试再次连接 */
    else if((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_DISCONNECTED)) {

        ESP_LOGI(LOGTEMP, "连接失败，尝试再次连接");
        ESP_ERROR_CHECK(esp_wifi_connect());
    }

    /* 连接成功。已获取AP分配的IP地址 */
    else if((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP)) {

        ip_event_got_ip_t *ipevent = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(LOGTEMP, "已成功连接到AP");
        ESP_LOGI(LOGTEMP, "获取的IP地址: "IPSTR"", IP2STR(&ipevent->ip_info.ip));
        ESP_LOGI(LOGTEMP, "获取的网关的IP: "IPSTR"", IP2STR(&ipevent->ip_info.gw));

        /* 通知客户端任务函数，STA已连接成功 */
        xTaskNotifyGive(g_xClientTaskHandle);
    }
}

/* 客户端初始化函数 */
void Client_Init(void)
{
    /* 创建默认事件循环 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 注册事件 */
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, Client_EventHandler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, Client_EventHandler, NULL, NULL);

    /* 初始化网卡底层配置 */
    ESP_ERROR_CHECK(esp_netif_init());

    /* 默认方式创建STA网卡 */
    esp_netif_t* pxNetifPort = esp_netif_create_default_wifi_sta();

    /* 设置静态IP */
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(pxNetifPort));

    esp_netif_ip_info_t xNetif_IPStruct = {

        .ip.addr = inet_addr(WIFI_STA_IP),
        .gw.addr = inet_addr(WIFI_STA_GW),
        .netmask.addr = inet_addr(WIFI_STA_NETMASK),
    };
    ESP_ERROR_CHECK(esp_netif_set_ip_info(pxNetifPort, &xNetif_IPStruct));

    /* 初始化WiFi的底层配置 */
    wifi_init_config_t xWiFi_InitStruct = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&xWiFi_InitStruct));

    /* 设置WiFi模式 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* 设置STA属性 */
    wifi_config_t xWiFi_ConfigStruct = {

        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &xWiFi_ConfigStruct));

    /* 启动WiFi */
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate((TaskFunction_t)Client_Task,
                "Client",
                1024*2,
                NULL,
                1,
                &g_xClientTaskHandle);
}

/* 客户端任务函数 */
void Client_Task(void *pvParameter)
{
    SERVER_CONNECT_STATE_E state = CONNECT_INIT;
    QueueHandle_t xQueue = *UartGet_ExchQueue();
    InputQueue_t xInput = *UartGet_InputQueue();

    while(1) {

        /* 等待STA连接成功 */
        if((state == CONNECT_INIT) || (state == CONNECT_OFFLINE_ERROR)) {

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            state = CONNECT_SUCCESS;
        }

        /* 创建socket */
        g_sockport = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if(g_sockport < 0) {
            ESP_LOGI(LOGTEMP, "创建socket失败");
            shutdown(g_sockport, SHUT_RDWR);
            close(g_sockport);
        }

        /* 绑定服务器信息，并连接服务器 */
        struct sockaddr_in xSockAddr_Struct = {

            .sin_family = AF_INET,
            .sin_port   = htons(6666),
            .sin_addr.s_addr   = inet_addr(WIFI_STA_GW),
        };
        int err = connect(g_sockport, (struct sockaddr*)&xSockAddr_Struct, sizeof(xSockAddr_Struct));
        if (err == 0){
            ESP_LOGI("CLIENT", "连接成功");
            state = CONNECT_SUCCESS;
        }else{
            ESP_LOGI("CLIENT", "连接失败，错误代码：%d", err);
            state = CONNECT_OFFLINE_NORMAL;
            shutdown(g_sockport, SHUT_RDWR);
            close(g_sockport);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        while(state == CONNECT_SUCCESS) {

            /* 接收服务器数据 */
            ESP_LOGI(LOGTEMP, "开始接收");
            int len = recv(g_sockport, g_receivebuff, sizeof(g_receivebuff), 0);
            ESP_LOGI(LOGTEMP, "recv len is %d", len);
            if(len == 0) {
                ESP_LOGI(LOGTEMP, "服务器正常下线");
                shutdown(g_sockport, SHUT_RDWR);
                close(g_sockport);
                state = CONNECT_OFFLINE_NORMAL;
                break;
            }
            else if(len < 0) {
                ESP_LOGI(LOGTEMP, "服务器非法下线");
                shutdown(g_sockport, SHUT_RDWR);
                close(g_sockport);
                state = CONNECT_OFFLINE_ERROR;
                break;
            }
            else {

                memcpy(xInput.iBuff, g_receivebuff, len);
                xInput.iLenght = len;
                if(xQueueSend(xQueue, &xInput, 0) == pdPASS) {

                    ESP_LOGI(LOGTEMP, "发送数据失败");
                }
            }
        }
    }
}


int Client_GetSocknum(void)
{
    return g_sockport;
}
