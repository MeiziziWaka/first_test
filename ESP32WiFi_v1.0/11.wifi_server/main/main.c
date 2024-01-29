#include <stdio.h>
#include "nvs_flash.h"

#include "Application/app_unittest.h"

void app_main(void)
{
    /* 初始化默认NVS分区 */
    ESP_ERROR_CHECK(nvs_flash_init());

    appUnitTest_Init();
}
