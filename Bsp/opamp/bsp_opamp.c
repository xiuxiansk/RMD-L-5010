#include "bsp_opamp.h" // 包含 OPAMP相关的 BSP（板级支持包）头文件
#include "stm32g4xx_hal_dac.h"
#include "stm32g4xx_hal_dac_ex.h"

/**
 * @brief 初始化 OPAMP BSP
 *
 * 此函数用于初始化 OPAMP 的相关设置。
 */
void opamp_bsp_init(void)
{
    // HAL_DACEx_SelfCalibrate(&hdac3, DAC_ChannelConfTypeDef * sConfig, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

    // 配置 OPAMP1
    // HAL_OPAMP_SelfCalibrate(&hopamp1);
    // HAL_OPAMP_SelfCalibrate(&hopamp2);
    // HAL_OPAMP_SelfCalibrate(&hopamp3);
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
}
