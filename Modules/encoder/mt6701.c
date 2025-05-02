#include "mt6701.h"

/**
 * @brief 编码器初始化函数
 *
 * 此函数用于初始化编码器相关参数和 SPI 外设。
 *
 * @param None
 */
ENCODER_DATA encoder_data = {
    .pole_pairs           = 0,     // 设置极对数为电机参数
    .encoder_offset       = 0,     // 设置电角度偏移量
    .dir                  = 0,     // 设置编码器旋转方向为顺时针
    .cnt                  = 0,     // 计数器
    .theta_acc            = 0.01f, // 设置角度累加步长为0.01弧度
    .count_in_cpr_        = 0,     // 原始值（整数型）
    .shadow_count_        = 0,     // 原始值累加计数（整数型）
    .pos_estimate_counts_ = 0.0f,  // 原始值累加计数（浮点型）
    .vel_estimate_counts_ = 0.0f,  // 原始值转速
    .pos_cpr_counts_      = 0.0f,  // 原始值（浮点型）
    .pos_estimate_        = 0.0f,  // 圈数n
    .vel_estimate_        = 0.0f,  // 速度rad/s
    .pos_cpr_             = 0.0f,  // 0-1
    .phase_               = 0.0f,  // 电角度
    .interpolation_       = 0.0f,  // 插值系数
    .elec_angle           = 0.0f,  // 电角度
    .mec_angle            = 0.0f,  // 机械角度
    .calib_valid          = false, // 表示校准的数据是否有效
};

/**
 * @brief 将角度归一化到 [0, 2π] 范围内
 *
 * 此函数将给定的角度值归一化到 [0, 2π] 范围内。
 *
 * @param angle 要归一化的角度
 */
float normalize_angle(float angle)
{
    float a = fmod(angle, M_2PI);    // 使用取余运算进行归一化
    return a >= 0 ? a : (a + M_2PI); // 确保返回值在 [0, 2π] 范围内
}

/**
 * @brief 从 MT6701 编码器读取原始角度数据
 *
 * 此函数读取 MT6701 编码器的原始角度值
 *
 * @param encoder 指向包含电机状态和控制数据的结构体指针
 */
bool mt6701_read_raw(ENCODER_DATA *encoder)
{
    uint8_t rx[3];

    MT6701_SPI_CS_L();
    if (HAL_SPI_Receive(&MT6701_SPI_Get_HSPI, rx, 3, 0xFFF) != HAL_OK) {
        MT6701_SPI_CS_H();
        goto TIMEOUT;
    }
    MT6701_SPI_CS_H();
    // 磁场状态检测
    encoder->mag_status = rx[2] >> 6;
    if (encoder->mag_status) {
        goto CHECK_ERR;
    }
    // 角度读取
    encoder->angle = (rx[0] << 6) | (rx[1] >> 2);
    return true;

CHECK_ERR:
    if (encoder->check_err_count < 0xFF) {
        encoder->check_err_count++;
    }
    return false;

TIMEOUT:
    if (encoder->rx_err_count < 0xFF) {
        encoder->rx_err_count++;
    }
    return false;
    // 奇偶校验
    // h_count = 0;
    // for (uint8_t j = 0; j < 16; j++) {
    //     if (rawAngle & (0x01 << j)) {
    //         h_count++;
    //     }
    // }
    // if (h_count & 0x01) {
    //     goto CHECK_ERR;
    // }
}

/**
 * @brief 将读取的原始值转化成实际角度和电角度
 *
 * 此函数将读取的原始值转化成实际角度和电角度。
 *
 * @param encoder 指向包含电机状态和控制数据的结构体指针
 */
void GetMotor_Angle(ENCODER_DATA *encoder)
{
    static const float pll_kp_        = 2.0f * ENCODER_PLL_BANDWIDTH;
    static const float pll_ki_        = 0.25f * SQ(pll_kp_);
    static const float snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * pll_ki_;
    if (mt6701_read_raw(encoder)) {
        if (encoder->dir == CW) {
            encoder->cnt = encoder->angle;
        } else {
            encoder->cnt = (ENCODER_CPR - encoder->angle);
        }
    }

    int delta_enc = encoder->cnt - encoder->count_in_cpr_;
    delta_enc     = mod(delta_enc, ENCODER_CPR);
    if (delta_enc > ENCODER_CPR_DIV) {
        delta_enc -= ENCODER_CPR;
    }

    encoder->shadow_count_ += delta_enc;
    encoder->count_in_cpr_ += delta_enc;
    encoder->count_in_cpr_ = mod(encoder->count_in_cpr_, ENCODER_CPR);

    encoder->count_in_cpr_ = encoder->cnt;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    encoder->pos_cpr_counts_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
    // discrete phase detector
    float delta_pos_counts     = (float)(encoder->shadow_count_ - (int32_t)floor(encoder->pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(encoder->count_in_cpr_ - (int32_t)floor(encoder->pos_cpr_counts_));
    delta_pos_cpr_counts       = wrap_pm(delta_pos_cpr_counts, ENCODER_CPR_DIV);
    // pll feedback
    encoder->pos_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_counts;
    encoder->pos_cpr_counts_ += CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_cpr_counts;
    encoder->pos_cpr_counts_ = fmodf_pos(encoder->pos_cpr_counts_, ENCODER_CPR_F);
    encoder->vel_estimate_counts_ += CURRENT_MEASURE_PERIOD * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (ABS(encoder->vel_estimate_counts_) < snap_threshold) // 100
    {
        encoder->vel_estimate_counts_ = 0.0f; // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel              = true;
    }

    // Outputs from Encoder for Controller
    encoder->pos_estimate_ = encoder->pos_estimate_counts_ / ENCODER_CPR_F;
    encoder->vel_estimate_ = encoder->vel_estimate_counts_ / ENCODER_CPR_F;
    encoder->pos_cpr_      = encoder->pos_cpr_counts_ / ENCODER_CPR_F;

    //// run encoder count interpolation
    int32_t corrected_enc = encoder->count_in_cpr_ - encoder->encoder_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        encoder->interpolation_ = 0.5f;
        // reset interpolation if encoder edge comes
        // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        encoder->interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        encoder->interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        encoder->interpolation_ += CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (encoder->interpolation_ > 1.0f)
            encoder->interpolation_ = 1.0f;
        if (encoder->interpolation_ < 0.0f)
            encoder->interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + encoder->interpolation_;

    //// compute electrical phase
    float elec_rad_per_enc = encoder->pole_pairs * M_2PI * (1.0f / ENCODER_CPR_F);
    float ph               = elec_rad_per_enc * interpolated_enc;
    encoder->phase_        = wrap_pm_pi(ph); // 电角度
    encoder->mec_angle     = normalize_angle(corrected_enc * (360.0f / ENCODER_CPR_F) * (M_PI / 180.0f));
    encoder->elec_angle    = normalize_angle(encoder->pole_pairs * encoder->mec_angle); // 电角度
}

/**
 * @brief 角度自增函数
 *
 * 根据电机的旋转方向和速度，计算并更新电机的电角度。
 *
 * @param encoder 指向电机数据结构的指针
 */
void Theta_ADD(ENCODER_DATA *encoder)
{
    encoder->elec_angle += encoder->theta_acc;
    if (encoder->elec_angle > M_2PI)
        encoder->elec_angle = 0.0f;
    else if (encoder->elec_angle < 0)
        encoder->elec_angle = M_2PI;
}