#ifndef __APP_UNITTEST_H
#define __APP_UNITTEST_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WORDTOBYTE(__VALUE__, __OFFSET__)	((__VALUE__ >> (__OFFSET__ << 3)) & 0xFF)
#define BYTETOWORD(__VALUE__, __OFFSET__)	((unsigned int)__VALUE__ << (__OFFSET__ << 3))

TaskHandle_t * UnitTest_GetTaskHandle(void);
void UintTest_Init(void);

#endif /* __APP_UNITTEST_H */
