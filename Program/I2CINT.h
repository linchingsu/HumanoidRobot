/*
*********************************************************************************************************
** 文件名称 ： I2CINT.h
** 功能说明 ： LPC2200系列芯片硬件I2C软件包。
** 使用说明 ： 主程序要配置好I2C总线接口(I2C引脚功能和I2C中断，并已使能I2C主模式)
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


