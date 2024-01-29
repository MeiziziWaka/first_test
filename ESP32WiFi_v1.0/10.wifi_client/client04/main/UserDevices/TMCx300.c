#include <string.h>
#include "TMCx300.h"
#include "Application/App_Unittest.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

/* The declaration of this variables and functions. */

/* 创建两个TMC设备 */
TMCx300_Device_t g_tTMCx300_Device[2] = {

    {
        .name = "TMC7300",
        .slave_addr = 0,
    },

    {
        .name = "TMC2300",
        .slave_addr = 1,
    }
};

/* 定义一个TMC设备的链表头 */
static pTMCx300_Device g_ptTMCx300_Device;

/**
  * @brief  The function is used to Insert linked list using plug-in - register TMC devices.
  * @retval None.
  */
/**
  * @brief	使用头插法插入链表--注册TMC设备
  * @retval None.
  */
void TMCxDevice_Register(pTMCx300_Device pDevice)
{
    pDevice->pnext = g_ptTMCx300_Device;
    g_ptTMCx300_Device = pDevice;
}

/**
  * @brief  The function is used to Add TMC devices to the list.
  * @retval None.
  */
/**
  * @brief  将TMC设备添加进链表
  * @retval None.
  */
void TMCxDevice_Add(void)
{
    for(int i = 0; i < (sizeof(g_tTMCx300_Device)/sizeof(g_tTMCx300_Device[0])); i++) {

        TMCxDevice_Register(&g_tTMCx300_Device[i]);
    }
}

/**
  * @brief  The function is used to Find the device by name and return the device address.
  * @retval None.
  */
/**
  * @brief  通过name查找设备，返回设备地址
  * @retval None.
  */
pTMCx300_Device TMCxDevice_Get(char *name)
{
    pTMCx300_Device pTemp = g_ptTMCx300_Device;
    while(pTemp) {

        if(strcmp(name, pTemp->name) == 0) return pTemp;
        else {
            pTemp = pTemp->pnext;
        }
    }
    return NULL;
}

/* 计算CRC校验码，8位CRC多项式用于检查读取和写入访问 */
void TMCx300_calcCRC(uint8_t* datagram, uint8_t datagramLength)
{
    int i,j;

    /* 定义指针指向要校验数据的最后字节 */
    uint8_t* crc = datagram + (datagramLength-1);
    uint8_t currentByte;
    *crc = 0;

    /* 对消息的所有字节执行 */
    for (i=0; i<(datagramLength-1); i++) {

        currentByte = datagram[i];
        for (j=0; j<8; j++) {
            if ((*crc >> 7) ^ (currentByte&0x01))
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }
            currentByte = currentByte >> 1;
        }
    }
}

void TMCx300_WriteInt(pTMCx300_Device pDevice)
{
    /* 定于一个局部缓存区，用于发送TMC通讯数据帧 */
	uint8_t uTemp[8];	
	TickType_t xTicksToWait = pdMS_TO_TICKS(100u);

    /* 配置数据帧 */
	uTemp[0] = 0x05;				            /* UART模式保留字节 */
	uTemp[1] = pDevice->slave_addr;				/* 从机地址: SLAVEADDR=(MS2,MS1) */
	uTemp[2] = pDevice->register_addr | 0x80;	/* 寄存器地址 */

	uTemp[3] = WORDTOBYTE(pDevice->value, 3);
	uTemp[4] = WORDTOBYTE(pDevice->value, 2);
	uTemp[5] = WORDTOBYTE(pDevice->value, 1);
	uTemp[6] = WORDTOBYTE(pDevice->value, 0);	/* 32 bit data */

    /* 计算CRC校验，并开始发送 */
	TMCx300_calcCRC(uTemp, sizeof(uTemp));
    uart_write_bytes(UART_NUM_1, uTemp, sizeof(uTemp));

	vTaskDelay(xTicksToWait);
}

void TMCx300_ReadInt(pTMCx300_Device pDevice)
{
	uint8_t ucTemp[4] = {0};
	TickType_t xTicksToWait = pdMS_TO_TICKS(100u);
	ucTemp[0] = 0x05;
	ucTemp[1] = 0x03;
	ucTemp[2] = pDevice->slave_addr;

	TMCx300_calcCRC(ucTemp, sizeof(ucTemp));
    uart_write_bytes(UART_NUM_1, ucTemp, sizeof(ucTemp));

	vTaskDelay(xTicksToWait);
}

void TMC2300_Init(void)
{
    pTMCx300_Device pTemp = TMCxDevice_Get("TMC2300");
	uint8_t IRUN  = 17;		    //调整电机运行电流，数值范围0...31，建议在16...31之间。调整这里可改变电机转动时的扭矩
	uint8_t IHOLD = 0x08;		//调整电机待机电流，数值范围0...31。调整这里可改变电机待机时的扭矩

    pTemp->register_addr = TMC2300_IHOLD_IRUN;
    pTemp->value = (uint32_t)(0x10000 | (IRUN<<8) | IHOLD);
	TMCx300_WriteInt(pTemp);

    pTemp->register_addr = TMC2300_CHOPCONF;
    pTemp->value = (uint32_t)0x13008001;
	TMCx300_WriteInt(pTemp);

    pTemp->register_addr = TMC2300_VACTUAL;
    pTemp->value = 2500;
	TMCx300_WriteInt(pTemp);
}


void TMC7300_Init(void)
{
    pTMCx300_Device pTemp = TMCxDevice_Get("TMC7300");

    pTemp->register_addr = TMC7300_GCONF;
    pTemp->value = 1;
    TMCx300_WriteInt(pTemp);

    pTemp->register_addr = TMC7300_SLAVECONF;
    pTemp->value = 2;
    pTemp->value <<= 8;
    TMCx300_WriteInt(pTemp);

    pTemp->register_addr = TMC7300_CURRENT_LIMIT;
    pTemp->value = 17;
    pTemp->value <<= 8;
    TMCx300_WriteInt(pTemp);

    pTemp->register_addr = TMC7300_PWM_AB;
    pTemp->value = 128;
    TMCx300_WriteInt(pTemp);
}

void TMC2300_RunVelocity(int32_t velocity)
{
    pTMCx300_Device pTemp = TMCxDevice_Get("TMC2300");
	pTemp->register_addr = TMC2300_VACTUAL;
    pTemp->value = velocity;
	TMCx300_WriteInt(pTemp);
}


void TMC7300_RunVelocity(int32_t velocity)
{
    pTMCx300_Device pTemp = TMCxDevice_Get("TMC7300");
    pTemp->register_addr = TMC7300_PWM_AB;
    pTemp->value = velocity;
    TMCx300_WriteInt(pTemp);
}
