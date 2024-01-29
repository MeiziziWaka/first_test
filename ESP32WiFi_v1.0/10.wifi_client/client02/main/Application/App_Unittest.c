#include "App_Unittest.h"
#include "esp_timer.h"

#include "UserDevices/User_Uart.h"
#include "UserDevices/User_Client.h"
#include "UserDevices/TMCx300.h"
#include "UserDevices/bsp_as5600.h"

#include "freertos/timers.h"

#include "esp_log.h"

/* The declaration of this variables and functions. */
TaskHandle_t g_xUnitTestTaskHandler;
esp_timer_handle_t g_xAngleTimer;

void UnitTest_Task(void *);
void UnitTest_ReceiveRegister(uint8_t , uint8_t , uint8_t *);
void UnitTest_ProgramRegister(uint8_t , uint8_t , uint8_t *, uint8_t );
void angletimer_Interrupt(void*);
void UnitTest_TimerInit(void);

/**
  * @brief  The function is used to get a task handle for a unit test.
  * @retval TaskHandle_t point.
  */
/**
  * @brief	获取单元测试的任务句柄
  * @retval None.
  */
TaskHandle_t * UnitTest_GetTaskHandle(void)
{
    return &g_xUnitTestTaskHandler;
}

/**
  * @brief  The function is used to initialize unit tests.
  * @retval None.
  */
/**
  * @brief	用于初始化单元测试
  * @retval None.
  */
void UintTest_Init(void)
{
    UartBuff_Init();

    /* 初始化客户端 */
    Client_Init();

    TMCxDevice_Add();
    TMC2300_Init();
    TMC7300_Init();

    ESP_ERROR_CHECK(iic_MasterInit());

    pInputAngle ptAngle = as5600Get_InputAngle();
    as5600Get_InitAngle(ptAngle);

    UnitTest_TimerInit();

    // xTimerCreate();

    xTaskCreate((TaskFunction_t)UnitTest_Task,
                "UnitTest",
                1024*2,
                NULL,
                1,
                &g_xUnitTestTaskHandler);
}

/**
  * @brief  The Task Functions for Unit Tests.
  * @retval None.
  */
/**
  * @brief	单元测试的任务函数
  * @retval None.
  */
void UnitTest_Task(void *pvParameter)
{
    pModbusMessageTypedef pxModbusMessage = UartGet_ModbusMes();
    while(1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        switch (pxModbusMessage->modbusRW)
        {
        case 0x03:  /* read */
            /* code */
            UnitTest_ReceiveRegister(pxModbusMessage->modbusAddress, pxModbusMessage->modbusCount, pxModbusMessage->SendBuffer);
            Uart_TransferBuf(pxModbusMessage);
            Uart_ResetBuf(pxModbusMessage);
            break;
        
        case 0x10:  /* write */
            /* code */
            UnitTest_ProgramRegister(pxModbusMessage->modbusAddress, pxModbusMessage->modbusCount, pxModbusMessage->RecvBuffer, pxModbusMessage->RecvBufferLength);
            Uart_TransferBuf(pxModbusMessage);
            Uart_ResetBuf(pxModbusMessage);
            break;

        default:
            break;
        }
    }
}

void UnitTest_ReceiveRegister(uint8_t addr, uint8_t length, uint8_t *pbuf)
{
    uint32_t temp = 0;
    for(uint8_t i = 0; i < length; i++) {

        switch (addr++)
        {
        case 0:
            temp = 0x01;
            break;

        case 3:
            /* code */
            pInputAngle ptAngle = as5600Get_InputAngle();
            float angle = 0.08789 * ptAngle->iAngle_add;
            temp = *(uint32_t*)&angle;
            break;

        default:
            break;
        }

        *(pbuf + 3 + (4*i)) = WORDTOBYTE(temp, 3);
		*(pbuf + 4 + (4*i)) = WORDTOBYTE(temp, 2);
		*(pbuf + 5 + (4*i)) = WORDTOBYTE(temp, 1);
		*(pbuf + 6 + (4*i)) = WORDTOBYTE(temp, 0);
    }

}


void UnitTest_ProgramRegister(uint8_t addr, uint8_t length, uint8_t *pbuf, uint8_t buflen)
{
    uint32_t temp = 0;
    /* 参数合法性检测 */
	if((length * 4 != *(pbuf + 0)) || (*(pbuf + 0) + 1 != buflen)) return;
    

    for(uint8_t i = 0; i < length; i++) {

        temp |= BYTETOWORD(*(pbuf + 1 + (4 * i)), 3);
        temp |= BYTETOWORD(*(pbuf + 2 + (4 * i)), 2);
        temp |= BYTETOWORD(*(pbuf + 3 + (4 * i)), 1);
        temp |= BYTETOWORD(*(pbuf + 4 + (4 * i)), 0);

        switch (addr)
        {
        case 0:
            /* code */
            break;
        
        case 1:
            /* code */
            TMC2300_RunVelocity(temp);
            break;

        case 2:
            /* code */
            TMC7300_RunVelocity(temp);
            break;

        default:
            break;
        }
    }
}

void UnitTest_TimerInit(void)
{
    // esp_timer_init();

    esp_timer_create_args_t timer = {

        .arg = NULL,
        .callback = angletimer_Interrupt,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "angle_timer",
        .skip_unhandled_events = false,
    };
    esp_timer_create(&timer, &g_xAngleTimer);
    esp_timer_start_periodic(g_xAngleTimer, 10e3);
    ESP_LOGI("UNITTEST", "定时器启动");
}


void angletimer_Interrupt(void* arg)
{
    pInputAngle ptAngle = as5600Get_InputAngle();
    as5600Get_ZeroSign(ptAngle);
    as5600Get_AddAngle(ptAngle);
    // ESP_LOGI("ANGLE", "角度:%ld", ptAngle->iAngle_add);
}
