#include <string.h>
#include "esp_log.h"
#include "UserDevices/User_Uart.h"
#include "UserDevices/User_Client.h"
#include "Application/App_Unittest.h"

#include "driver/uart.h"
#include "sys/socket.h"

/* The declaration of this variables and functions. */
uint8_t g_uartreceivebuff[1024];
static QueueHandle_t g_uartqueue;

static QueueHandle_t g_exchqueue;
static InputQueue_t g_xInputqueue;

static ModbusMessageTypedef_t g_xModbusMessage = {
    .modbusSlave = 1,
};

void UartBuff_Task(void *pvParameter);

/**
  * @brief  The Function is used to Initialization function of UART.
  * @retval None.
  */
/**
  * @brief	UART的初始化函数
  * @retval None.
  */
void UartBuff_Init(void)
{
    uart_config_t xUartConfig_Struct = {

        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .parity     = UART_PARITY_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .stop_bits  = UART_STOP_BITS_1,
    };
    ESP_LOGI("UART", "注册");
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, sizeof(g_uartreceivebuff), 0, 5, &g_uartqueue, 0));
    ESP_LOGI("UART", "配置");
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &xUartConfig_Struct));
    ESP_LOGI("UART", "设置引脚");
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 18, 19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    xTaskCreate((TaskFunction_t)UartBuff_Task,
                "Uart_buff",
                1024*2,
                NULL,
                1,
                NULL);

    g_exchqueue = xQueueCreate(5, sizeof(g_xInputqueue));

    xTaskCreate((TaskFunction_t)UartExch_Task,
                "Uart_exch",
                1024*2,
                &g_xModbusMessage,
                1,
                NULL);
}

/**
  * @brief  The Function is used to Event Handling Task Function of UART.
  * @retval None.
  */
/**
  * @brief	UART的事件处理任务函数
  * @retval None.
  */
void UartBuff_Task(void *pvParameter)
{
    // TickType_t xTicksToWait = pdMS_TO_TICKS(1000ul);
    uart_event_t event;
    while(1) {

        if(xQueueReceive(g_uartqueue, &event, portMAX_DELAY)) {

            switch (event.type)
            {
            case UART_DATA:
                ESP_LOGI("UART", "接收到了%d长度的数据", event.size);
                // uart_read_bytes(UART_NUM_1, g_uartreceivebuff, event.size, portMAX_DELAY);
                // uart_write_bytes(UART_NUM_1, g_uartreceivebuff, event.size);
                break;
            
            case UART_BUFFER_FULL:
                ESP_LOGI("UART", "RX缓冲区满了");
                break;

            default:
                break;
            }
        }
    }
}

/**
  * @brief  The Function is used to Get the address of the UART cache queue.
  * @retval QueueHandle_t point.
  */
/**
  * @brief	获取UART缓存队列的地址
  * @retval None.
  */
QueueHandle_t * UartGet_ExchQueue(void)
{
    return &g_exchqueue;
}

pInputQueue_t UartGet_InputQueue(void)
{
    return &g_xInputqueue;
}

pModbusMessageTypedef UartGet_ModbusMes(void)
{
    return &g_xModbusMessage;
}

/**
  * @brief  The Function is used to To calculate CRC check value.
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

/**
  * @brief  The task function of UART cache data.
  * @retval QueueHandle_t point.
  */
/**
  * @brief	UART缓存数据的任务函数
  * @retval None.
  */
void UartExch_Task(void *pvParameter)
{
    ModbusMessageTypedef_t *pxModbusMessage = (ModbusMessageTypedef_t *)pvParameter;
    TaskHandle_t *pTaskHandle = UnitTest_GetTaskHandle();

    while(1) {

        /* 等待队列通知 */
        if(xQueueReceive(g_exchqueue, &g_xInputqueue, portMAX_DELAY)) {

            /* 计算 CRC 校验码 */
            uint16_t crcvalue = UartExch_calcCRC(g_xInputqueue.iBuff, g_xInputqueue.iLenght - 2);

            /* 校验数据 */
            if(((g_xInputqueue.iBuff[0] == g_xModbusMessage.modbusSlave) || (g_xInputqueue.iBuff[0] == 0)) \
            && ((g_xInputqueue.iBuff[g_xInputqueue.iLenght - 2] == (crcvalue / 256)) || (g_xInputqueue.iBuff[g_xInputqueue.iLenght - 2] == 0xAA)) \
            && ((g_xInputqueue.iBuff[g_xInputqueue.iLenght - 1] == (crcvalue % 256)) || (g_xInputqueue.iBuff[g_xInputqueue.iLenght - 1] == 0xAA)))
            {

#if 1
                /* 这里进行缓存操作，便于下次接收 */
			    /* 功能码 */
				pxModbusMessage->modbusRW = g_xInputqueue.iBuff[1];

			    /* 寄存器地址 + 寄存器长度 */
				pxModbusMessage->modbusAddress = ((unsigned short)g_xInputqueue.iBuff[2] * 256) + g_xInputqueue.iBuff[3];
				pxModbusMessage->modbusCount = ((unsigned short)g_xInputqueue.iBuff[4] * 256) + g_xInputqueue.iBuff[5];

			    /* 计数值 - 转存数据的长度 */
				pxModbusMessage->RecvBufferLength = g_xInputqueue.iLenght - 8;

			    /* 缓存数据并唤醒处理函数 */
				memcpy(pxModbusMessage->RecvBuffer, g_xInputqueue.iBuff + 6, pxModbusMessage->RecvBufferLength);
                ESP_LOGI(LOGTEMP, "唤醒单元测试任务");
                xTaskNotifyGive(*pTaskHandle);
#endif

            }

            /* 有通知进来，就清除队列数据 */
            memset(g_xInputqueue.iBuff, 0, g_xInputqueue.iLenght);
        }
    }
}

void Uart_TransferBuf(pModbusMessageTypedef message)
{
    /* 参数合法性检测 */
	if(message == NULL) return;
	unsigned short temp = 0;
    int socknum = Client_GetSocknum();
    int err = 0;

    /* 判断读写标志 */
	switch (message->modbusRW)
	{
		case 0x03:	/* read */
			*(message->SendBuffer + 0) = g_xModbusMessage.modbusSlave;
			*(message->SendBuffer + 1) = 0x03;
			*(message->SendBuffer + 2) = message->modbusCount * 4;
			message->SendBufferLength = *(message->SendBuffer + 2) + 5;
			temp = UartExch_calcCRC(message->SendBuffer, message->SendBufferLength - 2);
			*(message->SendBuffer + (message->SendBufferLength - 2)) = temp / 256;
			*(message->SendBuffer + (message->SendBufferLength - 1)) = temp % 256;
			err = send(socknum, message->SendBuffer, message->SendBufferLength, 0);
            if(err < 0) {ESP_LOGI(LOGTEMP, "发送数据失败");}
 			break;
		
		case 0x10:	/* write */
			message->SendBufferLength = 8;
			*(message->SendBuffer + 0) = g_xModbusMessage.modbusSlave;
			*(message->SendBuffer + 1) = 0x10;
			*(message->SendBuffer + 2) = message->modbusAddress / 256;
			*(message->SendBuffer + 3) = message->modbusAddress % 256;
			*(message->SendBuffer + 4) = message->modbusCount / 256;
			*(message->SendBuffer + 5) = message->modbusCount % 256;
			temp = UartExch_calcCRC(message->SendBuffer, message->SendBufferLength - 2);
			*(message->SendBuffer + (message->SendBufferLength - 2)) = temp / 256;
			*(message->SendBuffer + (message->SendBufferLength - 1)) = temp % 256;
			err = send(socknum, message->SendBuffer, message->SendBufferLength, 0);
            if(err < 0) {ESP_LOGI(LOGTEMP, "发送数据失败");}
			break;
			
		default:
			break;
	}
}

void Uart_ResetBuf(pModbusMessageTypedef message)
{
    memset(message->RecvBuffer, 0, message->RecvBufferLength);
	memset(message->SendBuffer, 0, message->SendBufferLength);
	message->modbusAddress = 0;
	message->modbusCount = 0;
	message->modbusRW = 0;
	message->RecvBufferLength = 0;
	message->SendBufferLength = 0;
}
