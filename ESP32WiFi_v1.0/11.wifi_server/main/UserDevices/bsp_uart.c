#include <string.h>
#include "bsp_uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sys/socket.h"

#include "UserDevices/User_Server.h"
#include "Application/app_unittest.h"

/* The declaration of this variables and functions. */
uint8_t g_uartreceivebuf[1024];
QueueHandle_t g_tUartqueue;

void UartExch_Buffer(uint8_t* , uint8_t , pModbusMessageTypedef );
void UartEvent_SendClient(uint8_t *, uint8_t );
void UartEvent_Task(void *);
char *UartEvent_GetSocketname(uint8_t );

/**
  * @brief  The Function is used to Initialization of UART peripherals.
  * @retval None.
  */
/**
  * @brief	UART外设初始化
  * @retval None.
  */
void bspUart_Init(void)
{
    /* 设置UART配置参数 */
    uart_config_t conf = {

        .baud_rate = 115200,                        /* 波特率 */
        .data_bits = UART_DATA_8_BITS,              /* 数据位 */
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,      /* 硬件流控制 */
        .parity    = UART_PARITY_DISABLE,           /* 奇偶校验位 */
        .rx_flow_ctrl_thresh = 0,                   /* 硬件流控制阈值 */
        .source_clk = UART_SCLK_DEFAULT,            /* 时钟源 */
        .stop_bits = UART_STOP_BITS_1,              /* 停止位 */
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &conf));

    /* 将UART外设分配给GPIO引脚 */
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 18, 19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* 安装UART驱动程序并将UART设置为默认配置 */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, sizeof(g_uartreceivebuf), 0, 10, &g_tUartqueue, 0));

    xTaskCreate((TaskFunction_t)UartEvent_Task,
                "uart-event",
                1024*2,
                NULL,
                1,
                NULL);
}

/**
  * @brief  The Function is used to Initialization of UART peripherals.
  * @retval None.
  */
/**
  * @brief	UART外设初始化
  * @retval None.
  */
void UartEvent_Task(void *pvParameter)
{
    /* UART事件队列中的事件结构体 */
    uart_event_t event;
    while(1) {

        /* 读取队列，没有数据时阻塞运行 */
        if(xQueueReceive(g_tUartqueue, &event, portMAX_DELAY) == pdTRUE) {

            switch (event.type)
            {
            case UART_DATA:         /* UART数据事件 */
                /* code */
                ESP_LOGI("UART", "接收到%d个数据", event.size);
                // UartExch_Buffer(g_uartreceivebuf, event.size, &g_tModbusMessgae);
                uart_read_bytes(UART_NUM_1, g_uartreceivebuf, event.size, portMAX_DELAY);
                UartEvent_SendClient(g_uartreceivebuf, event.size);
                break;
            
            case UART_BUFFER_FULL:  /* UART RX缓冲区满事件 */
                /* code */
                ESP_LOGI("UART", "Rx缓冲区已满");
                break;

            default:
                break;
            }
        }
    }
}

#if 0
/**
  * @brief  The Function is used to Get the modbus Data Frame Structure Address.
  * @retval pModbusMessageTypedef.
  */
/**
  * @brief	获取modbus数据帧结构体地址
  * @retval pModbusMessageTypedef.
  */
pModbusMessageTypedef UartEventGet_Message(void)
{
    return &g_tModbusMessgae;
}
#endif

/**
  * @brief  The Function is used to calculate CRC check value.
  * @retval None.
  */
/**
  * @brief	计算CRC校验值
  * @retval None.
  */
uint16_t UartExch_calcCRC(uint8_t *pbuff, uint8_t length)
{
    uint16_t retvalue = 0, crcvalue = 0xFFFF;
    for(int i = 0; i < length; i++) {

        crcvalue ^= *(pbuff + i);
        for(int j = 0; j < 8; j++) {

            if(crcvalue & 0x0001) {

                crcvalue >>= 1;
                crcvalue ^= 0xA001;
            }
            else crcvalue >>= 1;
        }
    }
    retvalue |= (crcvalue % 256) << 8;
    retvalue |= (crcvalue / 256);
    return retvalue;
}

#if 0
/**
  * @brief  The Function is used to To UART exchange buffer data.
  * @retval None.
  */
/**
  * @brief	UART转存数据
  * @retval None.
  */
void UartExch_Buffer(uint8_t *pbuf, uint8_t length, pModbusMessageTypedef message)
{
    TaskHandle_t * ptask = UnittestGet_TaskHandle();
    /* 计算CRC校验位，进行数据验证 */
    uint16_t crc = UartExch_calcCRC(pbuf, length - 2);
    
    if(((pbuf[0] == message->modbusSlave) || (pbuf[0] == 0)) && ((pbuf[length - 2] == (crc / 256)) || (pbuf[length - 2] == 0xAA)) \
    && ((pbuf[length - 1] == (crc % 256)) || (pbuf[length - 1] == 0xAA)))
    {
        /* 功能码 */
        message->modbusRW = pbuf[1];

        /* 寄存器地址与长度 */
        message->modbusRegAddr = (unsigned short)(pbuf[2] * 256) + pbuf[3];
        message->modbusRegLen  = (unsigned short)(pbuf[4] * 256) + pbuf[5];

        /* 缓存数据的长度，去掉地址码，功能码，寄存器地址，寄存器长度，CRC校验码 */
        message->RecvBufferLength = length - 8;
        memcpy(message->RecvBuffer, pbuf + 6, message->RecvBufferLength);

        /* 唤醒单元测试任务 */
        xTaskNotifyGive(*ptask);
    }

    /* 有数据进入，就需要进行清除操作 */
    memset(pbuf, 0, length);
}
#endif

/**
  * @brief  The Function is used to UART triggers a data event that sends data to the client.
  * @retval None.
  */
/**
  * @brief	UART触发数据事件，将数据发送给客户端
  * @retval None.
  */
void UartEvent_SendClient(uint8_t *pbuf, uint8_t len)
{
    char * tempname = NULL;
    
    /* 计算CRC校验位，进行数据验证 */
    uint16_t crc = UartExch_calcCRC(pbuf, len - 2);
    if(((pbuf[len - 2] == (crc / 256)) || (pbuf[len - 2] == 0xAA)) && ((pbuf[len - 1] == (crc % 256)) || (pbuf[len - 1] == 0xAA)))
    {
        if(pbuf[0] != 0) {

            tempname = UartEvent_GetSocketname(pbuf[0]);
            pInputSocket pSock = GetInputSocket(tempname);
            int err = send(pSock->isocketnum, pbuf, len, 0);
            if(err < 0) ESP_LOGI("UART", "send失败");
            else {
                if(xSemaphoreGive(pSock->isocket_Sem) == pdTRUE);
                else ESP_LOGI("UART", "释放信号量失败");
            }
        }
        else {

            for(uint8_t i = 1; i <= 2; i++) {

                tempname = UartEvent_GetSocketname(i);
                pInputSocket pSock = GetInputSocket(tempname);
                int err = send(pSock->isocketnum, pbuf, len, 0);
                if(err < 0) ESP_LOGI("UART", "send失败");
                else {
                    if(xSemaphoreGive(pSock->isocket_Sem) == pdTRUE);
                    else ESP_LOGI("UART", "释放信号量失败");
                }
            }
        }
    }

    /* 有数据进入，就需要进行清除操作 */
    memset(pbuf, 0, len);
}

/**
  * @brief  The Function is used to Get the name of the input socket.
  * @retval char *.
  */
/**
  * @brief	获取输入socket的名称
  * @retval char *.
  */
char *UartEvent_GetSocketname(uint8_t socknum)
{
    char *temp = NULL;
    switch (socknum)
    {
    case 1:
        /* code */
        temp = "client01";
        break;
    
    case 2:
        /* code */
        temp = "client02";
        break;

    case 3:
        /* code */
        temp = "client03";
        break;

    case 4:
        /* code */
        temp = "client04";
        break;
    
    default:
        break;
    }
    return temp;
}
