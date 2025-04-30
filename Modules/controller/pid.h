#ifndef PID_H_
#define PID_H_

#include "general_def.h" // ����ͨ�ö����ͷ�ļ�

enum PID_MODE
{
  PID_POSITION = 0, // λ��ʽ
  PID_DELTA         // ����ʽ
};

typedef struct
{
  uint8_t mode;

  float Kp;
  float Ki;
  float Kd;

  float max_out;  // ������
  float max_iout; // ���������

  float set;
  float fdb;

  float out;
  float Pout;
  float Iout;
  float Dout;
  float Dbuf[3];  // ΢���� 0���� 1��һ�� 2���ϴ�
  float error[3]; // ����� 0���� 1��һ�� 2���ϴ�

} PidTypeDef;

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 *                 PID_DELTA: delta pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
/**
 * @brief          pid struct data init
 * @param[out]     pid: PID�ṹ����ָ��
 * @param[in]      mode: PID_POSITION:��ͨPID
 *                 PID_DELTA: ���PID
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid������
 * @param[in]      max_iout: pid���������
 * @retval         none
 */
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
/**
 * @brief          pid����
 * @param[out]     pid: PID�ṹ����ָ��
 * @param[in]      ref: ��������
 * @param[in]      set: �趨ֵ
 * @retval         pid���
 */
extern float PID_Calc(PidTypeDef *pid, float ref, float set);

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
/**
 * @brief          pid ������
 * @param[out]     pid: PID�ṹ����ָ��
 * @retval         none
 */
extern void PID_clear(PidTypeDef *pid);

#endif /* CODE_INCLUDE_PID_H_ */
