/****************************************Copyright (c)**************************************************
**                               Guangzou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			main.c
** Last modified Date:  2004-09-16
** Last Version:		1.0
** Descriptions:		The main() function example template
**
**------------------------------------------------------------------------------------------------------
** Created by:			Chenmingji
** Created date:		2004-09-16
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Chenxibing
** Modified date:		2005-01-17
** Version:
** Descriptions:		UART0ͨѶʵ�飬��ѯ��ʽ��
**
********************************************************************************************************/
#include "config.h"




#define	UART0_BPS	38400				// ����0ͨѶ������
#define	UART1_BPS	115200				// ����1ͨѶ������
#define RE485   IO0CLR=(IO0CLR& ~(0x01<<18)) |  (0x01<<18) 
#define RE485OFF   IO0SET=(IO0SET& ~(0x01<<18)) |  (0x01<<18)

#define DE485   IO0SET=(IO0SET& ~(0x01<<18)) |  (0x01<<18)
#define DE485OFF   IO0CLR=(IO0CLR& ~(0x01<<18)) |  (0x01<<18) 

/*
*********************************************************************************************************
** �������� ��DelayNS()
** �������� ���������ʱ��
** ��ڲ��� ��dly	��ʱ������ֵԽ����ʱԽ��
** ���ڲ��� ����
*********************************************************************************************************
*/
void DelayNS(uint32 dly)
{
	uint32 i;
	for ( ; dly>0; dly--)
		for (i=50000; i>0; i--);
}


/*
*********************************************************************************************************
** �������� ��UART0_Init()
** �������� �����ڳ�ʼ��������Ϊ8λ����λ��1λֹͣλ������żУ�飬������115200��
** ��ڲ��� ����
** ���ڲ��� ����
*********************************************************************************************************
*/
void UART0_Init (void)
{
	uint16 Fdiv;
	PINSEL0 =(PINSEL0 & 0xfffffff0)| 0x00000005;				// SER I/O P0.0 P0.1 connect to UART0
	U0LCR = 0x83;						// DLAB=1,�������ò�����
	Fdiv  = (Fpclk / 16) / UART0_BPS;	// ���ò�����
	U0DLM = Fdiv / 256;
	U0DLL = Fdiv % 256;
	U0LCR = 0x03;
}

void UART1_Init (void)
{
	uint16 Fdiv;
	PINSEL0 =(PINSEL0 & ~(0x0f<<16))| 0x05<<16;				// SER I/O P0.8 P0.9 connect to UART1
	PINSEL1 =(PINSEL1 & ~(0x03<<4)) | 0x00<<4;        // Set I/O P0.18 as GPIO;  
	IO0DIR=IO0DIR|(0x01<<18);
	U1LCR = 0x83;						// DLAB=1,�������ò�����
	Fdiv  = (Fpclk / 16) / UART1_BPS;	// ���ò�����
	U1DLM = Fdiv / 256;
	U1DLL = Fdiv % 256;
	U1LCR = 0x03;
}

void URAT1_Clr(void)
{
	U1FCR=U1FCR & ~(0x01<<1);
}

/*
*********************************************************************************************************
** �������� ��UART0_GetByte()
** �������� ���Ӵ��ڽ���1�ֽ����ݣ�ʹ�ò�ѯ��ʽ���ա�
** ��ڲ��� ����
** ���ڲ��� �����յ�������
*********************************************************************************************************
*/
uint8 UART0_GetByte (void)
{
	uint8 rcv_dat;
	
	while ((U0LSR & 0x01) == 0);
	rcv_dat = U0RBR;
	
	return (rcv_dat);	
}

uint8 UART1_GetByte (void)
{
	uint8 rcv_dat;
	
	while ((U1LSR & 0x01) == 0);
	rcv_dat = U1RBR;
	
	return (rcv_dat);	
}

uint8 UART1_GetByte485 (void)
{
	uint8 rcv_dat;
	RE485;
	while ((U1LSR & 0x01) == 0);
	rcv_dat = U1RBR;
	RE485OFF;
	return (rcv_dat);	
}

uint8 UART1_GetByte485C (void)
{
	uint8 rcv_dat;
	int i=0;
	RE485;
	while ((U1LSR & 0x01) == 0){
    i++;
		DelayNS(1);
		if(i>10)break;
  };
	
	if(i>100)rcv_dat = 123;
	else rcv_dat=U1RBR;
	RE485OFF;
	return (rcv_dat);	
}
/*
*********************************************************************************************************
** �������� ��UART0_GetStr()
** �������� ���Ӵ��ڽ���
** ��ڲ��� ��	s	ָ��������������ָ��
**				n	���յĸ���
** ���ڲ��� ��	��
*********************************************************************************************************
*/
void UART0_GetStr (uint8 *s, uint32 n)
{
	for ( ; n>0; n--)
	{
		*s++ = UART0_GetByte();
	}
}

void UART1_GetStr (uint8 *s, uint32 n)
{
	for ( ; n>0; n--)
	{
		*s++ = UART1_GetByte();
	}
}

void UART1_GetStr485 (uint8 *s, uint32 n)
{
	for ( ; n>0; n--)
	{
		*s++ = UART1_GetByte485();
	}
}


/*
*********************************************************************************************************
** �������� ��UART0_SendByte()
** �������� ���򴮿ڷ����ֽ����ݣ����ȴ�������ϣ���ѯ��ʽ��
** ��ڲ��� ��dat	Ҫ���͵�����
** ���ڲ��� ����
*********************************************************************************************************
*/
void UART0_SendByte (uint8 dat)
{
	U0THR = dat;
	while ((U0LSR & 0x40) == 0);		// �ȴ����ݷ������
}

void UART1_SendByte (uint8 dat)
{
	U1THR = dat;
	while ((U1LSR & 0x40) == 0);		// �ȴ����ݷ������
}

void UART1_SendByte485 (uint8 dat)
{
	U1THR = dat;
	DE485;
	while ((U1LSR & 0x40) == 0);		// �ȴ����ݷ������
	DE485OFF;
}
/*
*********************************************************************************************************
** �������� ��UART0_SendStr()
** �������� ���򴮿ڷ���һ�ַ���
** ��ڲ��� ��str	Ҫ���͵��ַ�����ָ��
** ���ڲ��� ����
*********************************************************************************************************
*/
void UART0_SendStr (uint8 const *str)
{
	while (1)
	{
		if (*str == '\0')	break;		// �������������˳�
		UART0_SendByte(*str++);			// ��������
	}
}

void UART1_SendStr (uint8 const *str)
{
	while (1)
	{
		if (*str == '\0')	break;		// �������������˳�
		UART1_SendByte(*str++);			// ��������
	}
}

void UART1_SendStr485 (uint8 const *str, uint8 n)
{
	
	for ( ; n>0; n--)
	{
		UART1_SendByte485(*str++);			// ��������
	}
}


uint8 ClrStr(uint8 * str)
{
	int i;
	int num;
	num=strlen(str);
	for(i=0; i<num; i++)
	{
		str[i]='\0';
	}
	return(num);
	
}
