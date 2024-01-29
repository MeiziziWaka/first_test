#include "app_unittest.h"

#include "UserDevices/bsp_uart.h"
#include "UserDevices/User_Server.h"

/* The declaration of this variables and functions. */
// static TaskHandle_t g_tUnittestTask;

void UnitTest_Task(void *);

#if 0
/**
  * @brief  The Function is used to get a task handle for a unit test.
  * @retval None.
  */
/**
  * @brief	获取单元测试的任务句柄
  * @retval None.
  */
TaskHandle_t* UnittestGet_TaskHandle(void)
{
    return &g_tUnittestTask;
}
#endif

/**
  * @brief  The Function is used to Unit Test Initialization.
  * @retval None.
  */
/**
  * @brief	单元测试初始化
  * @retval None.
  */
void appUnitTest_Init(void)
{
    AddInputSocket();
    userServer_Init();
    
    bspUart_Init();
}

#if 0
/**
  * @brief  The Function is used to Unit Test Initialization.
  * @retval None.
  */
/**
  * @brief	单元测试初始化
  * @retval None.
  */
void UnitTest_Task(void *pvParameter)
{
    pModbusMessageTypedef pMessage = UartEventGet_Message();
    while(1) {

        /* 等待任务通知 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 判断数据帧的功能码 */
        switch (pMessage->modbusRW)
        {
        case 0x03:  /* read */
            /* code */

            break;
        
        case 0x10:  /* write */
            /* code */
            break;

        default:
            break;
        }

    }
}

void UnitTest_ReceiveRegister(uint8_t slave, uint8_t addr, uint8_t len)
{
    /* 四个从机客户端 */
    switch (slave)
    {
    case 0:
        /* code */
        break;
    
    case 1:
        /* code */
        break;
    
    case 2:
        /* code */
        break;
    
    case 3:
        /* code */
        break;
    
    default:
        break;
    }
}
#endif
