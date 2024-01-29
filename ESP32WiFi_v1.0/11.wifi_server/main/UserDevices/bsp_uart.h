#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdio.h>

typedef struct{

	unsigned char  	modbusSlave;		/* 从机地址 */
	unsigned char	modbusBuad;			/* 从机波特率 */
	unsigned char  	modbusRW;			/* 读写选择 */
	unsigned short 	modbusRegAddr;		/* 寄存器地址 */
    unsigned char	RecvBuffer[100];    // 数据缓存
    unsigned char   RecvBufferLength;   // 数据缓存长度
    unsigned char   SendBuffer[100];    // 数据缓存
    unsigned char   SendBufferLength;   // 数据缓存长度
	unsigned char 	modbusRegLen;		/* 计数 读取或写入的位数 */
	void*   modbusPdata;				/* 数据指针: 写操作将指针指向的数据写入从机，读操作将读到的数据写入指针 */
}ModbusMessageTypedef_t, *pModbusMessageTypedef;


void bspUart_Init(void);

#endif /* __BSP_UART_H */
