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
** Descriptions:		UART0通讯实验，查询方式。
**
********************************************************************************************************/
#include "config.h"




#define	UART0_BPS	38400				// 串口0通讯波特率
#define	UART1_BPS	115200				// 串口1通讯波特率
#define RE485   IO0CLR=(IO0CLR& ~(0x01<<18)) |  (0x01<<18) 
#define RE485OFF   IO0SET=(IO0SET& ~(0x01<<18)) |  (0x01<<18)

#define DE485   IO0SET=(IO0SET& ~(0x01<<18)) |  (0x01<<18)
#define DE485OFF   IO0CLR=(IO0CLR& ~(0x01<<18)) |  (0x01<<18) 

/*
*********************************************************************************************************
** 函数名称 ：DelayNS()
** 函数功能 ：长软件延时。
** 入口参数 ：dly	延时参数，值越大，延时越久
** 出口参数 ：无
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
** 函数名称 ：UART0_Init()
** 函数功能 ：串口初始化，设置为8位数据位，1位停止位，无奇偶校验，波特率115200。
** 入口参数 ：无
** 出口参数 ：无
*********************************************************************************************************
*/
void UART0_Init (void)
{
	uint16 Fdiv;
	PINSEL0 =(PINSEL0 & 0xfffffff0)| 0x00000005;				// SER I/O P0.0 P0.1 connect to UART0
	U0LCR = 0x83;						// DLAB=1,允许设置波特率
	Fdiv  = (Fpclk / 16) / UART0_BPS;	// 设置波特率
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
	U1LCR = 0x83;						// DLAB=1,允许设置波特率
	Fdiv  = (Fpclk / 16) / UART1_BPS;	// 设置波特率
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
** 函数名称 ：UART0_GetByte()
** 函数功能 ：从串口接收1字节数据，使用查询方式接收。
** 入口参数 ：无
** 出口参数 ：接收到的数据
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
** 函数名称 ：UART0_GetStr()
** 函数功能 ：从串口接收
** 入口参数 ：	s	指向接收数据数组的指针
**				n	接收的个数
** 出口参数 ：	无
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
** 函数名称 ：UART0_SendByte()
** 函数功能 ：向串口发送字节数据，并等待发送完毕，查询方式。
** 入口参数 ：dat	要发送的数据
** 出口参数 ：无
*********************************************************************************************************
*/
void UART0_SendByte (uint8 dat)
{
	U0THR = dat;
	while ((U0LSR & 0x40) == 0);		// 等待数据发送完毕
}

void UART1_SendByte (uint8 dat)
{
	U1THR = dat;
	while ((U1LSR & 0x40) == 0);		// 等待数据发送完毕
}

void UART1_SendByte485 (uint8 dat)
{
	U1THR = dat;
	DE485;
	while ((U1LSR & 0x40) == 0);		// 等待数据发送完毕
	DE485OFF;
}
/*
*********************************************************************************************************
** 函数名称 ：UART0_SendStr()
** 函数功能 ：向串口发送一字符串
** 入口参数 ：str	要发送的字符串的指针
** 出口参数 ：无
*********************************************************************************************************
*/
void UART0_SendStr (uint8 const *str)
{
	while (1)
	{
		if (*str == '\0')	break;		// 遇到结束符，退出
		UART0_SendByte(*str++);			// 发送数据
	}
}

void UART1_SendStr (uint8 const *str)
{
	while (1)
	{
		if (*str == '\0')	break;		// 遇到结束符，退出
		UART1_SendByte(*str++);			// 发送数据
	}
}

void UART1_SendStr485 (uint8 const *str, uint8 n)
{
	
	for ( ; n>0; n--)
	{
		UART1_SendByte485(*str++);			// 发送数据
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
