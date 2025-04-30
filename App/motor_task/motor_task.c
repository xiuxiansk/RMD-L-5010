#include "motor_task.h"
#include "FOCMotor.h"

#define ADJUST_EN 0 // 调参使能

/**
 * @brief ADC2 电流采样完成回调函数
 *
 * 电流采样频率24khz
 *
 * @param hadc 传入的 ADC2
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &ADC_HSPI) {
#if ADJUST_EN
        GetMotorADC2PhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
#else
        GetMotorADC2PhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
        MotorStateTask(&motor_data);
#endif
    }
}
