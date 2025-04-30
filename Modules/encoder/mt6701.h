#pragma once
#include "main.h"

// SPI句柄定义
#define MT6701_SPI_Get_HSPI (hspi3) // 获取SPI句柄

// 延时相关定义
#define MT6701_MAX_DELAY  (10) // 延时最大值（单位：毫秒）

#define MT6701_SPI_CS_L() HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET)
#define MT6701_SPI_CS_H() HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET)

// 常量定义
#define ENCODER_PLL_BANDWIDTH 2000.0f
#define ENCODER_CPR           16384u   // 编码器分辨率u
#define ENCODER_CPR_F         16384.0f // 编码器分辨率f
#define ENCODER_CPR_DIV       (ENCODER_CPR >> 1)
#define MAX_ANGLE             360.0f // 360度为一圈
#define MAX_ANGLE_HALF        180.0f // 180度

// 方向枚举类型定义
typedef enum {
    CW      = 1,  // 顺时针方向
    CCW     = -1, // 逆时针方向
    UNKNOWN = 0   // 未知或无效状态
} Direction;

typedef struct
{
    uint32_t angle;
    uint8_t mag_status;
    uint8_t rx_err_count;
    uint8_t check_err_count;

    int32_t offset_lut[128]; // 用于磁编码器线性化的查找表
    int32_t count;           // 磁编码器的当前计数值（在一个完整编码器周期内的计数值）

    // 角度信息
    uint8_t pole_pairs; // 电机极对数，用于计算电角度
    int cnt;
    int raw; // 磁编码器的原始计数值
    int dir; // 磁编码器的旋转方向，+1 表示顺时针，-1 表示逆时针
    int pos_abs_;
    int count_in_cpr_;
    int count_in_cpr_prev;
    int shadow_count_;
    int EncoderInit_Flag;
    float pos_estimate_counts_;
    float vel_estimate_counts_;
    float pos_cpr_counts_;
    float pos_estimate_;
    float vel_estimate_;
    float pos_cpr_;
    float phase_;
    float interpolation_;
    int calib_valid; // (Auto)
    float mec_angle;
    float elec_angle;     // 电角度，对应于机械角度的电机转子位置（根据电机的极对数计算得到）
    float encoder_offset; // offset
    float speed;          // 速度
    float last_angle;     // 上一次的角度

    // 速度信息
    float theta_acc; // 电角度目标增量
} ENCODER_DATA;

extern ENCODER_DATA encoder_data;

float normalize_angle(float angle);         // 角度归一化函数
void GetMotor_Angle(ENCODER_DATA *encoder); // 更新编码器数据函数
void Theta_ADD(ENCODER_DATA *encoder);      // 电角度增量计算函数