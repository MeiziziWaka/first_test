#include <stdio.h>
#include "nvs_flash.h"

#include "Application/App_Unittest.h"

/* main task */
void app_main(void)
{
    /* WiFi 会用 nvs 存储两个部分：1.WiFi配置信息；2.天线的调配信息 */
    ESP_ERROR_CHECK(nvs_flash_init());

    UintTest_Init();
}
