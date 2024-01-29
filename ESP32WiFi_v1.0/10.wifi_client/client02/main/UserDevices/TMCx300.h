#ifndef __TMCx300_H
#define __TMCx300_H

#include <stdio.h>

typedef struct TMCx300_Device {

    char *name;
    unsigned char slave_addr;
    unsigned char register_addr;
    unsigned int  value;
    struct TMCx300_Device *pnext;

}TMCx300_Device_t, *pTMCx300_Device;

// General Config Registers
#define TMC2300_GCONF 		0x00
#define TCM2300_GSTAT 		0x01
#define TCM2300_IFCNT 		0x02
#define TCM2300_SLAVECONF 	0x03
#define TCM2300_IOIN 		0x06

//Velocity Dependent Driver Feature
#define TMC2300_IHOLD_IRUN 	0x10
#define TCM2300_TPOWERDOWN 	0x11
#define TMC2300_TSTEP 		0x12
#define TMC2300_VACTUAL 	0x22

//StallGuard Control
#define TMC2300_TCOOLTHRS 	0x14
#define TMC2300_SGTHRS 		0x40
#define TMC2300_SG_VALUE 	0x41
#define TMC2300_COOLCONF 	0x42

#define TMC2300_CHOPCONF	0x6C

/* 1.一般寄存器 */
#define TMC7300_GCONF          0x00
#define TMC7300_GSTAT          0x01
#define TMC7300_IFCNT          0x02
#define TMC7300_SLAVECONF      0x03
#define TMC7300_IOIN           0x06

/* 2.电机控制寄存器 */
#define TMC7300_CURRENT_LIMIT  0x10
#define TMC7300_PWM_AB         0x22

/* 3.斩波器控制寄存器 */
#define TMC7300_CHOPCONF       0x6C
#define TMC7300_DRVSTATUS      0x6F
#define TMC7300_PWMCONF        0x70


void TMCx300_ReadInt(pTMCx300_Device pDevice);
void TMCxDevice_Add(void);
void TMC2300_Init(void);
void TMC7300_Init(void);
void TMC2300_RunVelocity(int32_t velocity);
void TMC7300_RunVelocity(int32_t velocity);

#endif /* __TMCx300_H */
