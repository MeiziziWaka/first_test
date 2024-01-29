#include "driver/i2c.h"
#include "bsp_as5600.h"

#define IIC_MASTER_FREQ_HZ          400000
#define IIC_MASTER_SCL_IO           8
#define IIC_MASTER_SDA_IO           9

/* The declaration of this variables and functions. */
InputAngle_t g_tInputAngle = {
    .name = "as5600",
    .pNext = NULL,
};

/**
  * @brief  The function is used to Initialize the host mode of the iic.
  * @retval None.
  */
/**
  * @brief	初始化iic的主机模式
  * @retval None.
  */
esp_err_t iic_MasterInit(void)
{
    i2c_config_t conf = {

        .mode       = I2C_MODE_MASTER,              /* 选择工作模式 */
        .clk_flags  = I2C_SCLK_SRC_FLAG_FOR_NOMAL,  /* 时钟源 */
        .master.clk_speed = IIC_MASTER_FREQ_HZ,     /* 时钟频率 */
        .scl_io_num = IIC_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_io_num = IIC_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,        /* 允许上拉 */
    };

    /* 配置驱动程序 */
    int err = i2c_param_config(I2C_NUM_0, &conf);
    if(err != ESP_OK) return err;

    /* 安装iic驱动 */
    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    return err;
}

/**
  * @brief  The function is used to Get the pointer address of the as5600 angle.
  * @retval None.
  */
/**
  * @brief	获取as5600角度的指针地址
  * @retval None.
  */
pInputAngle as5600Get_InputAngle(void)
{
    return &g_tInputAngle;
}

/**
  * @brief  The function is used to iic reads multiple data.
  * @retval None.
  */
/**
  * @brief	iic读取多个数据
  * @retval None.
  */
void iic_ReadBytes(uint8_t addr, uint8_t *pbuf, uint8_t len)
{
    /* 用给定的缓冲区创建并初始化I2C命令列表 */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (__AS5600_ADDR<<1) | WRITE_BIT, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (__AS5600_ADDR<<1) | READ_BIT, true);
    while(len--) {

        i2c_master_read_byte(cmd, pbuf++, (len == 0)?I2C_MASTER_NACK:I2C_MASTER_ACK);
    }
    i2c_master_stop(cmd);

    /* 在master模式下发送I2C总线上的所有队列命令 */
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);

    /* 释放I2C命令列表 */
    i2c_cmd_link_delete(cmd);
}

/**
  * @brief  The function is used to Read the current angle value from the as5600 peripheral.
  * @retval None.
  */
/**
  * @brief	从as5600外设读取当前角度值.
  * @retval None.
  */
uint16_t as5600Get_RAWAngle(void)
{
    uint8_t temp[2] = {0};
    iic_ReadBytes(AS5600_RAW_ANGLE, temp, sizeof(temp));
    return ((uint16_t)temp[0] * 256) + temp[1];
}

/**
  * @brief  The function is used to Initial angle value of Settings.
  * @retval None.
  */
/**
  * @brief	设置初始角度值.
  * @retval None.
  */
void as5600Get_InitAngle(pInputAngle ptAngle)
{
    if(!ptAngle) return;
    uint32_t temp = 0;
    uint8_t i;

    /* 舍弃刚开始的数据 */
    for(i = 0; i < 20; i++) as5600Get_RAWAngle();

    /* 滤波 */
    for(i = 0; i < 20; i++) temp += as5600Get_RAWAngle();
    ptAngle->iAngle_init = temp / 20;
    ptAngle->iAngle_last = ptAngle->iAngle_init;
    ptAngle->iAngle_new  = ptAngle->iAngle_init;
}

/**
  * @brief  The function is used to get the cumulative angle value.
  * @retval None.
  */
/**
  * @brief	获取累加角度值.
  * @retval None.
  */
void as5600Get_AddAngle(pInputAngle ptAngle)
{
    /* 获取角度缓存区地址，并进行合法性检测 */
    if(!ptAngle) return;

    /* 判断过零点标志 */
    if(ptAngle->iAngle_zerosign == 0) {

        /* 正转：累计角度 = 新值-初始值 */
        if(ptAngle->iAngle_new >= ptAngle->iAngle_init) {

            ptAngle->iAngle_add = ptAngle->iAngle_new - ptAngle->iAngle_init;
            ptAngle->iAngle_dir = INPUTANGLE_RIGHT;
        }

        /* 反转：累计角度 = 初始值-新值 */
        else {

            ptAngle->iAngle_add = ptAngle->iAngle_new - ptAngle->iAngle_init;
            ptAngle->iAngle_dir = INPUTANGLE_LEFT;
        }
    }

    /* 经过一次及以上零点位置后 */
    else {

        if(ptAngle->iAngle_zerosign > 0) ptAngle->iAngle_dir = INPUTANGLE_RIGHT;
        else ptAngle->iAngle_dir = INPUTANGLE_LEFT;

        ptAngle->iAngle_add = ptAngle->iAngle_new - ptAngle->iAngle_init + (ptAngle->iAngle_zerosign * 4096);
    }
}

/**
  * @brief  The function is used to Get the zero-crossing mark.
  * @retval None.
  */
/**
  * @brief	获取过零点标志.
  * @retval None.
  */
void as5600Get_ZeroSign(pInputAngle ptAngle)
{
    if(!ptAngle) return;
    int temp = 0;

    /* 访问临界资源，关闭调度器 */
    // vTaskSuspendAll();
    ptAngle->iAngle_new = as5600Get_RAWAngle();
    // xTaskResumeAll();

    /* 通过角度获取过零点标志 */
    if(ptAngle->iAngle_new >= ptAngle->iAngle_last) {

        temp = ptAngle->iAngle_new - ptAngle->iAngle_last;
        if(abs(temp) > 2048) ptAngle->iAngle_zerosign--;
    }
    else {

        temp = ptAngle->iAngle_last - ptAngle->iAngle_new;
        if(abs(temp) > 2048) ptAngle->iAngle_zerosign++;
    }
    ptAngle->iAngle_last = ptAngle->iAngle_new;
}
