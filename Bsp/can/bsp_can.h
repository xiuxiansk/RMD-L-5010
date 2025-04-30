#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "general_def.h"
#include "main.h"
#include "fdcan.h"

// ����ܹ�֧�ֵ�CAN�豸��
#define FDCAN_MX_REGISTER_CNT 6 // �������ȡ����CAN���ߵĸ���
#define DEVICE_CAN_CNT 1        // ���ݰ����趨

typedef struct fdcaninstance
{
  FDCAN_HandleTypeDef *fdcan_handle; // can���
  FDCAN_TxHeaderTypeDef txconf;      // CAN���ķ�������
  uint32_t tx_id;                    // ����id
  uint32_t tx_mailbox;               // CAN��Ϣ����������
  uint8_t tx_buff[8];                // ���ͻ���,������Ϣ���ȿ���ͨ��CANSetDLC()�趨,���Ϊ8
  uint8_t rx_buff[8];                // ���ջ���,�����Ϣ����Ϊ8
  uint32_t rx_id;                    // ����id
  uint8_t rx_len;                    // ���ճ���,����Ϊ0-8
  // ���յĻص�����,���ڽ������յ�������
  void (*fdcan_module_callback)(struct fdcaninstance *); // callback needs an instance to tell among registered ones
  void *id;                                              // ʹ��can�����ģ��ָ��(��idָ���ģ��ӵ�д�canʵ��,�Ǹ��ӹ�ϵ)
} FDCANInstance;

/* CANʵ����ʼ���ṹ��,���˽ṹ��ָ�봫��ע�ắ�� */
typedef struct
{
  FDCAN_HandleTypeDef *fdcan_handle;              // can���
  uint32_t tx_id;                                 // ����id
  uint32_t rx_id;                                 // ����id
  void (*fdcan_module_callback)(FDCANInstance *); // ����������ݵĻص�����
  void *id;                                       // ӵ��canʵ����ģ���ַ,�������ֲ�ͬ��ģ��(�������Ҫ�Ļ�),�������Ҫ���Բ�����
} FDCAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        ע��(��ʼ��)һ��canʵ��,��Ҫ�����ʼ�����õ�ָ��.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
FDCANInstance *FDCANRegister(FDCAN_Init_Config_s *config);

/**
 * @brief �޸�CAN���ͱ��ĵ�����֡����;ע����󳤶�Ϊ8,��û�н����޸ĵ�ʱ��,Ĭ�ϳ���Ϊ8
 *
 * @param _instance Ҫ�޸ĳ��ȵ�canʵ��
 * @param length    �趨����
 */
void FDCANSetDLC(FDCANInstance *_instance, uint8_t length);

/**
 * @brief transmit mesg through CAN device,ͨ��canʵ��������Ϣ
 *        ����ǰ��Ҫ��CANʵ����tx_buffд�뷢������
 *
 * @attention ��ʱʱ�䲻Ӧ�ó������ô˺��������������,����ᵼ����������
 *
 * @param timeout ��ʱʱ��,��λΪms;������Ϊus,��ø���ȷ�Ŀ���
 * @param _instance* can instance owned by module
 */
uint8_t FDCANTransmit(FDCANInstance *_instance, float timeout);

#endif
