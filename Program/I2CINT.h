/*
*********************************************************************************************************
** �ļ����� �� I2CINT.h
** ����˵�� �� LPC2200ϵ��оƬӲ��I2C�������
** ʹ��˵�� �� ������Ҫ���ú�I2C���߽ӿ�(I2C���Ź��ܺ�I2C�жϣ�����ʹ��I2C��ģʽ)
*********************************************************************************************************
*/

#ifndef  I2CINT_H
#define  I2CINT_H

#define	ONE_BYTE_SUBA	1
#define TWO_BYTE_SUBA	2
#define X_ADD_8_SUBA	3

extern uint8 IRcvByte(uint8 sla, uint8 *dat);
extern uint8 ISendByte(uint8 sla, uint8 dat);
extern uint8 I2C_ReadNByte (uint8 sla, uint32 suba_type, uint32 suba, uint8 *s, uint32 num);
extern uint8 I2C_WriteNByte(uint8 sla, uint8 suba_type, uint32 suba, uint8 *s, uint32 num);
extern void __irq IRQ_I2C(void);

#endif


