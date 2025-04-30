#ifndef _Motor_ADC_H__ // ��ֹ�ظ�����ͷ�ļ�
#define _Motor_ADC_H__

#include "main.h"

#define ADC_HSPI (hadc2) // ��ȡSPI���;

// ���� ADC ƫ�ýṹ�壬���ڴ洢������������ƫ��ֵ
typedef struct
{
    ADC_HandleTypeDef *hadc;    // ADC ���
    float Temp_Result;          // �¶�
    float Ia_offset;            // Ia ƫ��
    float Ib_offset;            // Ib ƫ��
    float Ic_offset;            // Ic ƫ��
    float current_offset_sum_a; // Ia ƫ�ú�
    float current_offset_sum_b; // Ib ƫ�ú�
    float current_offset_sum_c; // Ic ƫ�ú�
} CURRENT_DATA;

extern CURRENT_DATA current_data;

/* adcͨ�����ֶ��壬�����Լ���ʹ�������� */
typedef enum {
    adc1_ch1 = 0,
    adc1_ch2 = 1,
    adc1_ch3 = 2,
    adc1_ch4 = 3
} adc1_num;

typedef enum {
    adc2_ch12 = 0,
} adc2_num;

void GetTempNtc(uint16_t value_adc, float *value_temp);
uint16_t adc1_median_filter(uint8_t channel);
uint16_t adc1_avg_filter(uint8_t channel);
uint16_t adc2_median_filter(uint8_t channel);
uint16_t adc2_avg_filter(uint8_t channel);
float low_pass_filter(float input, float output);
float avg_filter(Queue *q, float value, uint8_t len);
#endif // _Motor_ADC_H__
