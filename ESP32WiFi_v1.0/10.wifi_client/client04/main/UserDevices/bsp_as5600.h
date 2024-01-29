#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include <stdio.h>

#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE                            /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                              /*!< I2C master read */
#define ACK_CHECK_EN 0x1                                      /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                                     /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                                           /*!< I2C ack value */
#define NACK_VAL 0x1                                          /*!< I2C nack value */

#define __AS5600_ADDR		0x36

/* 配置寄存器 */
#define AS5600_ZMCO			0x00			//烧录的次数
#define AS5600_ZPOS			0x01			//配置起始位置
#define AS5600_MPOS			0x03			//配置停止位置
#define AS5600_MANG			0x05			//配置最大角度
#define AS5600_CONF			0x07			//配置电源模式...看门狗

/* 输出寄存器 */
#define AS5600_RAW_ANGLE	0x0C			//原始角度
#define AS5600_ANGLE		0x0E			//缩放角度

/* 状态寄存器 */
#define AS5600_STATUS		0x0B			//状态寄存器
#define AS5600_AGC			0x1A			//增益强度
#define AS5600_MAGNTUDE		0x1B			//幅度寄存器

/* 烧录寄存器 */
#define AS5600_BURN			0xFF

typedef enum{

	INPUTANGLE_RIGHT = 0,
	INPUTANGLE_LEFT  = 1,
}INPUTANGLE_DIR;

typedef struct InputAngle{

	char *name;					/* 名称 */
	INPUTANGLE_DIR iAngle_dir;	/* 方向 */
	uint16_t iAngle_init;		/* 初始角度 */
	uint16_t iAngle_last;		/* 上次角度 */
	uint16_t iAngle_new;		/* 这次角度 */
	int32_t iAngle_add;		    /* 从初始角度开始的累计角度值 */
	int iAngle_zerosign;		/* 过零点标记：正向经过加一，反向减一 */
	struct InputAngle *pNext;	/* 指向下一个设备 */
}InputAngle_t, *pInputAngle;

esp_err_t iic_MasterInit(void);
pInputAngle as5600Get_InputAngle(void);
void as5600Get_InitAngle(pInputAngle ptAngle);
void as5600Get_AddAngle(pInputAngle ptAngle);
void as5600Get_ZeroSign(pInputAngle ptAngle);

#endif /* __BSP_AS5600_H */
