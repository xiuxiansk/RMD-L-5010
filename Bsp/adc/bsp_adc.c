#include "bsp_adc.h" // 包含 ADC 相关的 BSP（板级支持包）头文件
#include "my_queue.h"

/* adc_dma 数据存储数组
     用于存储 ADC1 的 DMA 转换结果。
     adc1_samples: 采样次数
     adc1_channel: 通道数
*/
/* adc_dma 数据存储数组*/
uint16_t adc1_dma_value[adc1_samples][adc1_channel];
uint16_t adc2_dma_value[adc2_samples][adc2_channel];
Queue adc2_queue[adc2_channel];
/**
 * @brief 初始化 ADC BSP
 *
 * 此函数用于初始化 ADC 的相关设置，包括启动 DMA 和注入模式。
 */
void adc_bsp_init(void)
{
    initQueue(adc2_queue);
    initQueue(&adc2_queue[1]);
    initQueue(&adc2_queue[2]);
    /**ADC自校验**/
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    /**开启ADC**/
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_dma_value, adc1_length);
#if ADC_INJECTED_ENABLE
    HAL_ADCEx_InjectedStart_IT(&hadc2);
#else // 如果不启用注入模式
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_dma_value, adc2_length);
#endif
}
