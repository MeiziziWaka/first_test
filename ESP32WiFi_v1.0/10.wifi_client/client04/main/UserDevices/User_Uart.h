#ifndef __USER_UART_H
#define __USER_UART_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define DEFAULT_BUF_LEN     100


typedef struct InputQueue{

    uint8_t iBuff[DEFAULT_BUF_LEN];
    uint8_t iLenght;
}InputQueue_t, *pInputQueue_t;

typedef struct{

	unsigned char  	modbusSlave;		/* 从机地址 */
	unsigned char	modbusBuad;			/* 从机波特率 */
	unsigned char  	modbusRW;			/* 读写选择 */
	unsigned short 	modbusAddress;		/* 寄存器地址 */
    unsigned char	RecvBuffer[100];    // 数据缓存
    unsigned char   RecvBufferLength;   // 数据缓存长度
    unsigned char   SendBuffer[100];    // 数据缓存
    unsigned char   SendBufferLength;   // 数据缓存长度
	unsigned char 	modbusCount;		/* 计数 读取或写入的位数 */
	void*   modbusPdata;				/* 数据指针: 写操作将指针指向的数据写入从机，读操作将读到的数据写入指针 */
}ModbusMessageTypedef_t, *pModbusMessageTypedef;


void UartBuff_Init(void);
void UartBuff_Task(void *pvParameter);

QueueHandle_t * UartGet_ExchQueue(void);
pInputQueue_t UartGet_InputQueue(void);
pModbusMessageTypedef UartGet_ModbusMes(void);

void UartExch_Task(void *pvParameter);
void Uart_TransferBuf(pModbusMessageTypedef message);
void Uart_ResetBuf(pModbusMessageTypedef message);

#endif /* __USER_UART_H */
