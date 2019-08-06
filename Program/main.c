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
** Modified by:
** Modified date:
** Version:
** Descriptions:
**
********************************************************************************************************/
#include "config.h"
#include "UART.h"
#include "math.h"
#include "IMU.h"
#include "target.c"
#include "I2CINT.c"

//LED IO
#define	  LED1_OFF   IO1SET=(IO1SET& (~(0x01<<16)) ) |  (0x01<<16)
#define	  LED1_ON    IO1CLR=(IO1CLR& (~(0x01<<16))) |  (0x01<<16)
#define	  LED2_OFF   IO1SET=(IO1SET& (~(0x01<<17))) |  (0x01<<17)
#define	  LED2_ON    IO1CLR=(IO1CLR& (~(0x01<<17))) |  (0x01<<17)
#define	  LED3_OFF   IO1SET=(IO1SET& (~(0x01<<18))) |  (0x01<<18)
#define	  LED3_ON    IO1CLR=(IO1CLR& (~(0x01<<18))) |  (0x01<<18)

#define LED1	  (0x01<<16) 
#define LED2	  (0x01<<17)
#define LED3	  (0x01<<18)

//BEE IO
#define	  BEE_OFF   IO0SET=(IO0SET& (~(0x01<<19)) ) |  (0x01<<19)
#define	  BEE_ON    IO0CLR=(IO0CLR& (~(0x01<<19))) |  (0x01<<19)

#define BEE	    (0x01<<19) 

//Key IO

#define   KEYs      ((IO1PIN>>19) & (0x00000003))  
#define   KeyUP      ( IO1PIN & (0x01<<20) )
#define   KeyDOWN    ( IO1PIN & (0x01<<19) ) 

//IR & SWITCH IO

#define IR1	  (0x01<<21) 
#define IR2	  (0x01<<23)
#define HURT1	  (0x01<<22)
#define SW2	  (0x01<<24)



//#define SWDA    ((IO1PIN) & 0x0000003F)         //P2.0 TO P 2.5
#define HURTD1    ((IO1PIN>>22) & 0x00000001)       //P1.22
#define SWD2    ((IO1PIN>>24) & 0x00000001)       //P1.24

#define	  HURT1_OFF   IO1SET=(IO1SET& ~(0x01<<22)) |  (0x01<<22)
#define	  HURT1_ON    IO1CLR=(IO1CLR& ~(0x01<<22)) |  (0x01<<22)



#define IRD1    ((IO1PIN>>21) & 0x00000001)       //P1.22
#define IRD2    ((IO1PIN>>23) & 0x00000001)       //P1.23






//Brake IO

#define BK1	  (0x01<<7) 
#define BK2	  (0x01<<21)
#define BK3	  (0x01<<11)
#define BK4	  (0x01<<17)

#define	  BRAKE4_OFF   IO0SET=(IO0SET& ~(0x01<<17)) |  (0x01<<17)
#define	  BRAKE4_ON    IO0CLR=(IO0CLR& ~(0x01<<17)) |  (0x01<<17)
#define	  BRAKE3_OFF   IO0SET=(IO0SET& ~(0x01<<11)) |  (0x01<<11)
#define	  BRAKE3_ON    IO0CLR=(IO0CLR& ~(0x01<<11)) |  (0x01<<11)

#define	  BRAKE2_OFF   IO0SET=(IO0SET& ~(0x01<<21)) |  (0x01<<21)
#define	  BRAKE2_ON    IO0CLR=(IO0CLR& ~(0x01<<21)) |  (0x01<<21)
#define	  BRAKE1_OFF   IO0SET=(IO0SET& ~(0x01<<7)) |  (0x01<<7)
#define	  BRAKE1_ON    IO0CLR=(IO0CLR& ~(0x01<<7)) |  (0x01<<7)

//AD CLK & PRECISION
#define ADCLK 4500000        //定义AD部件时钟频率
#define ADBIT 10             //定义BURST模式下的转换精度
#define ADBIT2 (10-ADBIT)


//Servo ID
#define Svo1 1
#define Svo2 2
#define Svo3 3
#define Svo4 4
#define Svo5 5
#define Svo6 6
#define Svo7 7
#define Svo8 8
#define Svo9 9
#define Svo10 10
#define Svo11 11
#define Svo12 12
#define Svo13 13
#define Svo14 14
#define Svo15 15
#define Svo16 16
#define Svo17 17
#define Svoo18 18
#define Svo19 19
#define Svo20 20

//Servo Initial Position
#define SvoIni1 -21
#define SvoIni2 830
#define SvoIni3 148
#define SvoIni4 50
#define SvoIni5 32
#define SvoIni6 -999
#define SvoIni7 52
#define SvoIni8 820

#define KneeADC 7.5



#define SERVO3  (0X01<<21)
#define USS     (0X01<<28)
#define AUTO_SWITCH1     (0X01<<6)
#define AUTO_SWITCH2     (0X01<<7)
#define TAPIC  20
#define TAROL  20
#define DelA   5
#define TURNANGLE 40
#define TURNTIME  1
#define SVRPOD   14

#define TAPIC2  20
#define TAROL2  0

#define EANGLE1 -180
#define EANGLE2 360

#define AngleLine 55.4   //0.1degree per 55.4 Line



const int angle_max=45;
const int angle_min=-45;
const int time_max=2;
const int time_min=1;
const double INV=0.002;   //姿态采样周期，2ms
const double uss_pm=0.1724;
const double pi=3.1415926;
const int wGyro=300;
const int SPEED=200;   //200cm/s
const int TSPD=700;  //偏航周期
const int CINV=100; //偏航舵补偿间隔 
const int PINV=200; //升降舵时间间隔
const int TINV=100; //偏航舵时间间隔   偏航舵时间-升降舵时间-偏航舵时间
const int PDEL=10;  //升降舵角度
const int TDEL=-20; //偏航舵角度

double scale;


int gx,gy,gz;
int turetemp;
uint8    str[200];







uint8 CheckKey(void)
{
   uint8 key,key2;
   key=KEYs;
   if(key==0x03) return(0);
   else
   {      
      switch(key)
	  {
	  case 0x01: 
	    key2=1;
		break;
	  case 0x02:
	  	key2=2;
		break;  

	  default:
	    key2=6;
	    break;
	  }
	  return(key2);  
   }

}   

void AD0_ini(void)
{
/* 进行ADC0模块设置，其中x<<n表示第n位设置为x(若x超过一位，则向高位顺延) */
	//引脚配置	
  PINSEL0=(PINSEL0 & ~(0x03<<8))| (0x03<<8);  //P0.4-AD0.6  
	PINSEL0=(PINSEL0 & ~(0x03<<10))| (0x03<<10); //P0.5-AD0.7  
	
	PINSEL1=(PINSEL1 & ~(0x03<<18))| (0x01<<18);  //P0.25-AD0.4
	PINSEL1=(PINSEL1 & ~(0x03<<24))| (0x01<<24);  //P0.28-AD0.1
	PINSEL1=(PINSEL1 & ~(0x03<<26))| (0x01<<26);  //P0.29-AD0.2
	PINSEL1=(PINSEL1 & ~(0x03<<28))| (0x01<<28);  //P0.30-AD0.3


   AD0CR = (0X00<<27)                   |    //EDGE
	        //(0X01<<24)                  |    //START
          (0x00<< 22)                  | 		// TEST1:0 = 00 ，正常工作模式(非测试模式)
	        (0x01<< 21)                  | 		// PDN = 1 ， 正常工作模式(非掉电转换模式)
					(ADBIT2<<17)                 |    //CLKS
					(0X00 << 16)                 |		// BURST = 0 ，软件控制转换操作
	        ((Fpclk / (ADCLK + 1)) << 8); 		// CLKDIV = Fpclk / 4500000 + 1 ，即转换时钟为4.5MHz

   DelayNS(10);								

}

void AD1_ini(void)
{
/* 进行ADC1模块设置，其中x<<n表示第n位设置为x(若x超过一位，则向高位顺延) */
	//引脚配置	
  
	PINSEL0=(PINSEL0 & ~(0x03<<12))| (0x03<<12);  //P0.6-AD1.0  
	PINSEL0=(PINSEL0 & ~(0x03<<20))| (0x03<<20);  //P0.10_AD1.2
	PINSEL0=(PINSEL0 & ~(0x03<<24))| (0x03<<24);  //P0.12_AD1.3
	PINSEL0=(PINSEL0 & ~(0x03<<26))| (0x03<<26);  //P0.13_AD1.4
	PINSEL0=(PINSEL0 & ~(0x03<<30))| (0x03<<30);  //P0.15-AD1.5
	PINSEL1=(PINSEL1 & ~(0x03<<12))| (0x01<<12);  //P0.22-AD1.7
	
   AD1CR = (0X00<<27)                   |    //EDGE
	        //(0X01<<24)                   |    //START
          (0x00<< 22)                  | 		// TEST1:0 = 00 ，正常工作模式(非测试模式)
	        (0x01<< 21)                  | 		// PDN = 1 ， 正常工作模式(非掉电转换模式)
					(ADBIT2<<17)                 |    //CLKS
					(0X00 << 16)                 |		// BURST = 0 ，软件控制转换操作
	        ((Fpclk / (ADCLK + 1)) << 8); 		// CLKDIV = Fpclk / 4500000 + 1 ，即转换时钟为4.5MHz

   DelayNS(10);								

}

//AD读取函数    输入值AD通道数 0~7
uint32 AD0_data(uint8 n)
{
  	uint32  ADC_Data;
    AD0CR = (AD0CR&0xFFFFFF00)|(0x01<<n)|(1 << 24);	// 切换通道并进行第一次转换
    while( (AD0GDR&0x80000000)==0 );			// 等待转换结束
    ADC_Data = AD0GDR;							// 读取ADC结果
    ADC_Data = (ADC_Data>>6) & 0x3FF;
    ADC_Data = ADC_Data * 3200;
    ADC_Data = ADC_Data / 1024;
	return(ADC_Data);
}

//AD读取函数    输入值AD通道数 0~7
uint32 AD1_data(uint8 n)
{
  	uint32  ADC_Data;
    AD1CR = (AD1CR&0xFFFFFF00)|(0x01<<n)|(1 << 24);	// 切换通道并进行第一次转换
    while( (AD1GDR&0x80000000)==0 );			// 等待转换结束
    ADC_Data = AD1GDR;							// 读取ADC结果
    ADC_Data = (ADC_Data>>6) & 0x3FF;
    ADC_Data = ADC_Data * 3200;
    ADC_Data = ADC_Data / 1024;
	return(ADC_Data);
}

/****************************************************************************************
Futaba 伺服电机驱动
电机重启
ID: 
      
*****************************************************************************************/
int RStartmotor485(unsigned char ID )
{
	unsigned char	sendbuf[9];
	unsigned char	sum;
	int				i;
	int				ret;
	//unsigned long	len;

  ret=1;

	// clear sendbuf
	for(i=0; i<9; i++)
	{
		 sendbuf[i]=0x00;
	}

	// set sendbuf
	
	sendbuf[0]  = (unsigned char)0xFA;				// Head1
	sendbuf[1]  = (unsigned char)0xAF;				// Head2
	sendbuf[2]  = (unsigned char)ID;			    // Servo_ID
	sendbuf[3]  = (unsigned char)0x20;				// Flag
	sendbuf[4]  = (unsigned char)0xff;				// Adr(0x24=36)
	sendbuf[5]  = (unsigned char)0x00;				// Len(4byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// Caculation Sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set Sum
  
  for( i=0; i<8; i++)
	{
		UART1_SendByte485(sendbuf[i]);	    // 
	}
	DelayNS(10);
	return ret;
	
}
/****************************************************************************************
Futaba 伺服电机驱动
启动/关闭伺服模式
sMode: 1 On
       0 Off
       2 Brake
*****************************************************************************************/


int RSTorqueOnOff485(short sMode, unsigned char ID )
{
	unsigned char	sendbuf[9];
	unsigned char	sum;
	int				i;
	int				ret;
	//unsigned long	len;

  ret=1;

	// clear sendbuf
	for(i=0; i<9; i++)
	{
		 sendbuf[i]=0x00;
	}

	// set sendbuf
	
	sendbuf[0]  = (unsigned char)0xFA;				// Head1
	sendbuf[1]  = (unsigned char)0xAF;				// Head2
	sendbuf[2]  = (unsigned char)ID;			    // Servo_ID
	sendbuf[3]  = (unsigned char)0x00;				// Flag
	sendbuf[4]  = (unsigned char)0x24;				// Adr(0x24=36)
	sendbuf[5]  = (unsigned char)0x01;				// Len(4byte)
	sendbuf[6]  = (unsigned char)0x01;				// Cnt
	sendbuf[7]  = (unsigned char)(sMode&0x00FF);	// Dat (ON/OFF) 
	
	// Caculation Sum
	sum = sendbuf[2];
	for( i = 3; i < 8; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[8] = sum;								// set Sum
  
  for( i=0; i<9; i++)
	{
		UART1_SendByte485(sendbuf[i]);	    // 
	}
	DelayNS(10);
	return ret;
	
}

/****************************************************************************************
Futaba 伺服电机驱动
控制角度旋转
sPos:   position  1=0.1 degree
sTime:  Rotation cost Time
*****************************************************************************************/
int RSMove485( short sPos, unsigned short sTime, unsigned char ID )
{
	unsigned char	sendbuf[12];
	unsigned char	sum;
	int				i;
	int				ret;

 
//******************Check Limit of the Joint	
	//if(sPos>1100)sPos=1100;   
	//if(sPos<-1100)sPos=-1100;
//******************************//
	
  ret=1;
	// clear sendbuf
	for(i=0; i<12; i++)
	{
		 sendbuf[i]=0x00;
	}

	// set sendbuf

	sendbuf[0]  = (unsigned char)0xFA;				    // Head1
	sendbuf[1]  = (unsigned char)0xAF;				    // Head2
	sendbuf[2]  = (unsigned char)ID;			  //Servo_ID
	sendbuf[3]  = (unsigned char)0x00;				    // Flag
	sendbuf[4]  = (unsigned char)0x1E;				    // Adr(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04;				    // Len(4byte)
	sendbuf[6]  = (unsigned char)0x01;				    // Cnt
	sendbuf[7]  = (unsigned char)(sPos&0x00FF);		    // Dat
	sendbuf[8]  = (unsigned char)((sPos&0xFF00)>>8);	// Dat
	sendbuf[9]  = (unsigned char)(sTime&0x00FF);	    // Dat
	sendbuf[10] = (unsigned char)((sTime&0xFF00)>>8);	// Dat
	// Caculation Sum
	sum = sendbuf[2];
	for( i = 3; i < 11; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[11] = sum;								// Set sum
  
	//Send Command
	for( i=0; i<12; i++)
	{
		UART1_SendByte485(sendbuf[i]);	    // set command
	}
	//DelayNS(10);
	return ret;
	
}



/*----------------------------------------------------------------------------*/
/*
 * short RSGetAngle(unsigned char ID)
 * Return angle  0.1 deg
 */
/******************************************************************************/
short RSGetAngle485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[10];
	unsigned char	sum;
	int				i;
	short			angle;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<10; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x0F;				// Flag(0x01 | 0x04<<1) 0x09
	sendbuf[4]  = (unsigned char)0x2A;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x02;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  UART1_GetStr485 (readbuf, 10);
  DelayNS(1);
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	angle=0;
	angle=angle|readbuf[8];
	angle=angle<<8;
	angle=angle|readbuf[7];

	return angle;
}

/*----------------------------------------------------------------------------*/
/*
 * short RSGetTime(unsigned char ID)
 * Return time  ms
 */
/******************************************************************************/
short RSGetTime485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[10];
	unsigned char	sum;
	int				i;
	short			time;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<10; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x0F;				// Flag(0x01 | 0x04<<1) 0x09
	sendbuf[4]  = (unsigned char)0x2C;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x02;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  UART1_GetStr485 (readbuf, 10);
  
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	time=0;
	time=time|readbuf[8];
	time=time<<8;
	time=time|readbuf[7];

	return time;
}

/*----------------------------------------------------------------------------*/
/*
 * short RSGetSpeed(unsigned char ID)
 * Return speed   deg/sec
 */
/******************************************************************************/
short RSGetSpeed485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[10];
	unsigned char	sum;
	int				i;
	short			speed;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<10; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x0F;				// Flag(0x01 | 0x04<<1) 0x09
	sendbuf[4]  = (unsigned char)0x2E;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x02;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  UART1_GetStr485 (readbuf, 10);
  
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	speed=0;
	speed=speed|readbuf[8];
	speed=speed<<8;
	speed=speed|readbuf[7];

	return speed;
}

/*----------------------------------------------------------------------------*/
/*
 * short RSGetLoad(unsigned char ID)
 * Return load  mA
 */
/******************************************************************************/
short RSGetLoad485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[10];
	unsigned char	sum;
	int				i;
	short			load;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<10; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x0F;				// Flag(0x01 | 0x04<<1) 0x09
	sendbuf[4]  = (unsigned char)0x30;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x02;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  UART1_GetStr485 (readbuf, 10);
  
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	load=0;
	load=load|readbuf[8];
	load=load<<8;
	load=load|readbuf[7];

	return load;
}

/*----------------------------------------------------------------------------*/
/*
 * short RSGetVoltage(unsigned char ID)
 * Return voltage  10mV
 */
/******************************************************************************/
short RSGetVoltage485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[10];
	unsigned char	sum;
	int				i;
	short			voltage;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<10; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x0F;				// Flag(0x01 | 0x04<<1) 0x09
	sendbuf[4]  = (unsigned char)0x34;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x02;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  UART1_GetStr485 (readbuf, 10);
  
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	voltage=0;
	voltage=voltage|readbuf[8];
	voltage=voltage<<8;
	voltage=voltage|readbuf[7];

	return voltage;
}

/*----------------------------------------------------------------------------*/
/*
 * short RSGetTem(unsigned char ID)
 * Return tem 
 */
/******************************************************************************/
short RSGetTem485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[10];
	unsigned char	sum;
	int				i;
	short			tem;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<10; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x0F;				// Flag(0x01 | 0x04<<1) 0x09
	sendbuf[4]  = (unsigned char)0x32;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x02;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  UART1_GetStr485 (readbuf, 10);
  
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	tem=0;
	tem=tem|readbuf[8];
	tem=tem<<8;
	tem=tem|readbuf[7];

	return tem;
}

/*----------------------------------------------------------------------------*/
/*
 * short RSGetTem(unsigned char ID)
 * Return ID 
 */
/******************************************************************************/
uint8 RSHello485( unsigned char ID )
{
	unsigned char	sendbuf[8];
	unsigned char	readbuf[2];
	unsigned char	sum;
	int				i;
	uint8			rID;

	// clear sendbuf
	for(i=0; i<8; i++)
	{
		 sendbuf[i]=0x00;
	}
	
	//Clear readbuf
	for(i=0; i<2; i++)
	{
		 readbuf[i]=0x00;
	}
	// set command

	sendbuf[0]  = (unsigned char)0xFA;				// Head-1
	sendbuf[1]  = (unsigned char)0xAF;				// Head-2
	sendbuf[2]  = (unsigned char)ID;			     // Servo ID
	sendbuf[3]  = (unsigned char)0x01;				// Flag
	sendbuf[4]  = (unsigned char)0x00;				// Adr(0x00)
	sendbuf[5]  = (unsigned char)0x00;				// Len(0byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// caculation sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set sum

	//Send Command
	UART1_SendStr485(sendbuf,8);
  //Receive Result
  readbuf[0]=UART1_GetByte485C();
  
	/*
  for( i=0; i<10; i++)
	{
		UARTEX_SendByte(1,readbuf[i]);
	}
	*/
	
	rID=readbuf[0];
	return rID;
}

//Check ServoID
uint8 RSGetID485(uint8 maxID)
{
	uint8 i;
	for(i=1;i<=maxID;i++)
	{
		if( RSHello485(i)==0x07)break;
	}
	if(i>maxID) return 0;
	else return i;
}

uint8 RSSetID485(uint8 NewID)
{
	uint8 OldID;
	unsigned char	sendbuf[9];
	unsigned char	sum;
	int				i;
	int				ret;
	//unsigned long	len;

	//Ask ID
	OldID=RSGetID485(30);
	
	
  ret=1;

	// clear sendbuf
	for(i=0; i<9; i++)
	{
		 sendbuf[i]=0x00;
	}

	// set sendbuf
	
	sendbuf[0]  = (unsigned char)0xFA;				// Head1
	sendbuf[1]  = (unsigned char)0xAF;				// Head2
	sendbuf[2]  = (unsigned char)OldID;			    // Servo_ID
	sendbuf[3]  = (unsigned char)0x00;				// Flag
	sendbuf[4]  = (unsigned char)0x04;				// Adr(0x24=36)
	sendbuf[5]  = (unsigned char)0x01;				// Len(4byte)
	sendbuf[6]  = (unsigned char)0x01;				// Cnt
	sendbuf[7]  = (unsigned char)NewID;	      // Dat (NewID) 
	
	// Caculation Sum
	sum = sendbuf[2];
	for( i = 3; i < 8; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[8] = sum;								// set Sum
  
  for( i=0; i<9; i++)
	{
		UART1_SendByte485(sendbuf[i]);	    // 
	}
	DelayNS(10);
	return ret;
}

uint8 RSWriteReset(uint8 NewID)
{
	unsigned char	sendbuf[9];
	unsigned char	sum;
	int				i;
	int				ret;
	//unsigned long	len;


  ret=1;

	// clear sendbuf
	for(i=0; i<9; i++)
	{
		 sendbuf[i]=0x00;
	}

	// set sendbuf
	
	sendbuf[0]  = (unsigned char)0xFA;				// Head1
	sendbuf[1]  = (unsigned char)0xAF;				// Head2
	sendbuf[2]  = (unsigned char)NewID;			    // Servo_ID
	sendbuf[3]  = (unsigned char)(0x20|0x40);				// Flag
	sendbuf[4]  = (unsigned char)0xFF;				// Adr(0x24=36)
	sendbuf[5]  = (unsigned char)0x00;				// Len(4byte)
	sendbuf[6]  = (unsigned char)0x00;				// Cnt
	
	// Caculation Sum
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// set Sum
  
  for( i=0; i<8; i++)
	{
		UART1_SendByte485(sendbuf[i]);	    // 
	}
	DelayNS(10);
	return ret;
}

/**************************************Smart motor Driver*****************************/


/*************************************************************************************/
int Write_SmtMotor(uint8 address, uint8  *str, uint8 echeck )
{
	uint8 adr=address+128;
	DelayNS(2);
	UART0_SendByte(adr); //Send address
	UART0_SendStr(str);
	UART0_SendByte('\r');  //回车
	return(echeck);
}

void Read_SmtMotor(uint8  *str)
{
	int i,j;
	i=0;
	j=0;
	//while(UART0_RcvByte()!=0x0d);  //In case motor used ECHO
	
	while(j!=1)
	{
		str[i]=UART0_GetByte();
		if(str[i]==0x0d) {j++;}

		i++;
		
	}
	
}

int Get_SmtMotorP(uint8 address)  //Get setted relatived distance
{ 
	uint8 strx[30];
	int32 data; 
	Write_SmtMotor(address, "RP", 0);
	Read_SmtMotor(strx);
	return(atoi(strx));
}

int Get_SmtMotorD(uint8 address)  //Get setted accelerate
{
	uint8 strx[30];
	int32 data; 
	Write_SmtMotor(address, "RD", 0);
	Read_SmtMotor(strx);
	return(atoi(strx));
}
int Get_SmtMotorV(uint8 address)  //Get setted accelerate
{
	uint8 strx[30];
	int32 data; 
	Write_SmtMotor(address, "RV", 0);
	Read_SmtMotor(strx);
	return(atoi(strx));
}
/*************************************************************************************/
/*************************************************************************************/




void PWM_INI()
{
	  PINSEL0 = (PINSEL0 & (~(0x03<<14)) ) | (0x02<<14);   //SET P0.7 as PWM2
	  PINSEL0 = (PINSEL0 & (~(0x03<<16)) ) | (0x02<<16);   //SET P0.8 as PWM4
	  PINSEL0 = (PINSEL0 & (~(0x03<<18)) ) | (0x02<<18);   //SET P0.9 as PWM6
	  PINSEL1 = (PINSEL1 & (~(0x03<<10)) ) | (0x01<<10);   //Set P0.21 as PWM5
	  
	  PWMPR=0x00;
	  PWMMR0=Fpclk/(1000/SVRPOD);

	  PWMPCR |=(0x01<<10) |(0x01<<12) |(0x01<<13) |(0x01<<14) ;// Enable EACH PWM CHANNEL OUTPUT
	  //PWMMCR=0x02;
	  PWMMCR |=(0x01<<1);

	  PWMMR5=PWMMR0/(SVRPOD/1.5);
	  PWMMR2=PWMMR2/(SVRPOD/1.5);
	  PWMMR4=PWMMR4/(SVRPOD/1.5);
	  PWMMR6=PWMMR6/(SVRPOD/1.5);

	  PWMLER|=((0x01)|(0x01<<5)|(0x01<<6)|(0x01<<4)|(0x01<<2));			   
	  PWMTCR=0x09;
}

void SetServo(int channel, double angle)
{

	switch(channel){

	case 2:
	  PWMMR2=PWMMR0/(SVRPOD/ (double)(time_min+(angle-angle_min)*scale) );
	  PWMLER|=((0x01)|(0x01<<2));
	break;
	case 4:
	  PWMMR4=PWMMR0/(SVRPOD/ (double)(time_min+(angle-angle_min)*scale) );
	  PWMLER|=((0x01)|(0x01<<4));
	break;
	case 5:
	  PWMMR5=PWMMR0/(SVRPOD/ (double)(time_min+(angle-angle_min)*scale) );
	  PWMLER|=((0x01)|(0x01<<5));
	break;
	case 6:
	  PWMMR6=PWMMR0/(SVRPOD/ (double)(time_min+(angle-angle_min)*scale) );
	  PWMLER|=((0x01)|(0x01<<6));
	break;
	default:
	break;
	}
}


/*********************************************************************************************************
**                            主程序
********************************************************************************************************/
int main (void)
{
  char key;		
	char rev_num;
	char str[32];
	char num[32];
	
	short Speed1;
	short Speed2;
	short Speed3;
	short Speed4;
	short Speed5;
	short Speed6;	
	short Speed7;	
	short Speed8;	
	
	short SAng;
	short SAng1;
	short SAng2;	
	short SAng3;
	short SAng4;
	short SAng5;
	short SAng6;	
	short SAng7;
	short SAng8;
	

	int32 KAngL;
	int32 KAngR;
	uint8 ID;
	uint8 NewID;
	
	
  int delaytime=0; // hurt delay 
	
	int DA1;   // read input SW2
	int DA2;
	
	
	int i;
	int32  SitPt1;
	int32  SitPt2;
	char Arm_FlagR=0;
	char Arm_FlagL=0;
	char Em_Flag=0;
	char KL_Flag=0;
	char KR_Flag=0;
	char Sd_Flag=0;
	char Cmd_Flag=0;
	char Sit_Flag=0;
	
	const int SevNum=10;
  int s;
	
	int32  Ang[11];
	uint8 SevOn[11];
	uint8 AllSevOn;
	
	char	arm_int;
	char int_flagR;
	char int_flagP;	
	
	int32 ADC0[8];
	int32 ADC1[8];
  int32 FilBuf[10];
	int32 FilBufP[10];
	int32 FilBuf_abs[10];
	int32 FilBuf_absP[10];   
	int32 FilBuf_Pabs[10];
	
	
	
	DelayNS(50);
	TargetResetInit();                //Chip start up set
  UART0_Init();						          // INITIA UART0
  UART1_Init();                     // INITIA UART1
	AD0_ini();                         // INITIA AD0
	AD1_ini();                         // INITIA AD1
  I2cInit(400000);						       // I2C初始化，400K
	SC16IS752_Init_ChA1_232 (); 
	SC16IS752_Init_ChB1_232 (); 
	

	
		
// IO INI
	PINSEL2= PINSEL2&  (~(0x01<<3));   //set P1.16-P1.25 as GPIO   
  
	IO1DIR=IO1DIR & (~IR1);
	IO1DIR=IO1DIR & (~IR2);
	IO1DIR=IO1DIR & (~SW2);
	
	IO1DIR=IO1DIR|HURT1;              //Set P1.22 as Output
	
	
	IO1DIR=IO1DIR|LED1;               //Set P1.16 as Output
  IO1DIR=IO1DIR|LED2;               //Set P1.17 as Output
  IO1DIR=IO1DIR|LED3;               //Set P1.18 as Output
	
	IO0DIR=IO0DIR|BK1;               //Set P0.07 as Output
	IO0DIR=IO0DIR|BK2;               //Set P0.21 as Output
	IO0DIR=IO0DIR|BK3;               //Set P0.11 as Output
	IO0DIR=IO0DIR|BK4;               //Set P0.17 as Output
	
	BRAKE1_OFF;       //Knee Joint On
  BRAKE2_OFF;
  //IO0DIR=IO0DIR|BEE;                //Set P0.19 as Output
	//IO0SET=IO0SET||BEE;
	 
	
	
	LED1_OFF; 
	LED2_OFF;
	LED3_OFF;
	
	
	
	SetLED(1,0);
	SetLED(0,0);
	DelayNS(10);
	SetLED(1,1);
	DelayNS(10);
	SetLED(0,1);
	DelayNS(10);
	SetLED(1,0);
	DelayNS(10);
	SetLED(0,0);
	DelayNS(10);
	SetLED(1,1);
	SetLED(0,1);
	
	rev_num=0;
	//UART1_SendByte485 (0x48);
	//UART1_SendByte485 (0x50);
	//RStartmotor485(1);
	
 	
/***************************************Set Servo ID**************************
	NewID=8;  //Please Input New ID
	sprintf(str, "ID =%1d \n", RSGetID485(30));
	UARTEX_SendStr(1,str);ClrStr(str);
	RSSetID485(NewID); RSWriteReset(NewID); DelayNS(100);
  sprintf(str, "NewID =%1d \n", RSGetID485(30));
	UARTEX_SendStr(1,str);ClrStr(str);
	
	RSTorqueOnOff485(1, NewID );
	RSMove485( 900, 200, NewID );
			UARTEX_SendStr(1,"Go ");
	do
	{
		//UARTEX_SendStr(1,"Read ");
		SAng=RSGetAngle485(NewID);DelayNS(10);
		sprintf(str, "Ang =%1d deg \n", SAng);
		UARTEX_SendStr(1,str); ClrStr(str);
  }while(abs(SAng-900)>2);
	LED1_ON;
	RSMove485( 0, 200, NewID );
	UARTEX_SendStr(1,"Go2 ");
	do
	{
		//UARTEX_SendStr(1,"Read ");
		SAng=RSGetAngle485(NewID);DelayNS(10);
		sprintf(str, "ang =%1d deg \n",SAng);
		UARTEX_SendStr(1,str); ClrStr(str);
  } while(abs(SAng)>2);
		
  RSTorqueOnOff485(0, NewID );
	LED2_ON;
	while(1);
/**********************************Test Robot ARM***********************************/

/*

{
    SAng=RSGetAngle485(8);DelayNS(30);
		sprintf(str, "Ang =%1d deg \n", SAng);
		UARTEX_SendStr(0,str); ClrStr(str);
}
*/
//***************************************  Flag Int  *******************************************//
 
	int_flagR=0;       // Flag for R int postion 
	int_flagP=0;       // Flag for P int postion 
  arm_int=0;        // arm returen to int 
  Arm_FlagR=0;
  Arm_FlagL=0;

//*******************************  Unlock motor  smart motor  *******************************************//
	

  Write_SmtMotor( 1, " EIGN(2) ", 1);   // position limitation 
  Write_SmtMotor( 1, " EIGN(3) ", 1);   // position limitation 
  Write_SmtMotor( 1, " ZS  ", 1);			

	Write_SmtMotor( 2, " EIGN(2) ", 2);   // position limitation 
  Write_SmtMotor( 2, " EIGN(3) ", 2);   // position limitation 
  Write_SmtMotor( 2, " ZS  ", 2);			
	
	
	Write_SmtMotor( 3, " UCI ", 3);   // position limitation 
  Write_SmtMotor( 3, " UDI ",3);   // position limitation 
  Write_SmtMotor( 3, " ZS  ", 3);

  Write_SmtMotor( 4, " UCI ", 4);   // position limitation 
  Write_SmtMotor( 4, " UDI ",4);   // position limitation 
  Write_SmtMotor( 4, " ZS  ", 4);

	
	Write_SmtMotor( 3, "BRKSRV", 3);   // break on
  Write_SmtMotor( 4, "BRKSRV", 4);   // break on 

	

//*******************************  Waist Pitch Int***************************************//

/*
while(int_flagP==0)
{
ADC1[5]=AD1_data(5)/10;
sprintf(str, "ADC5 %1d \n", ADC1[5]); 			
UARTEX_SendStr(1,str); 	 
	 
 if(ADC1[5]>150)   //          back side 
 {
		
   while(int_flagP==0)	
   {		 
	 ADC1[5]=AD1_data(5)/10;
	 sprintf(str, "ADC5 %1d \n", ADC1[5]); 			
   UARTEX_SendStr(1,str); 			
   
	 Write_SmtMotor( 2, "AT=100 ", 2 );	
	 Write_SmtMotor( 2, "VT=100000 ", 2 );	
   Write_SmtMotor( 2, "PRT=1000 ", 2 );	//D relatice distance
   Write_SmtMotor( 2, "G ", 2 ); 
		
		
	 if(ADC1[5]<151) 
	 { 
	 Write_SmtMotor( 2, "S ", 2 ); 	
	 UARTEX_SendStr(1,"B to F"); 
	 int_flagP=1;
	 }
   } 
 }	 
	
 if(ADC1[5]<150)   //          Front side
 {
		
   while(int_flagP==0)	
   {		 
	 ADC1[5]=AD1_data(5)/10;
	 sprintf(str, "ADC5 %1d \n", ADC1[5]); 			
   UARTEX_SendStr(1,str);			
   
	 Write_SmtMotor( 2, "AT=100 ", 2 );	
	 Write_SmtMotor( 2, "VT=100000 ", 2 );	
   Write_SmtMotor( 2, "PRT=-1000 ", 2 );	//D relatice distance
   Write_SmtMotor( 2, "G ", 2 ); 
		
		
	 if(ADC1[5]>149) 
	 { 
	 Write_SmtMotor( 2, "S ", 2 ); 	
	 UARTEX_SendStr(1,"F to B"); 
	 int_flagP=1;
	 }
   } 
 }	 
	 

if(ADC1[5]==150)  int_flagP=1;
 
	
}
*/


//****************************Check RC Motor 1-10 until All checked***********************//
  //Clear Found RC motor Flag 
	
	
  AllSevOn=0;
	for(i=1; i<=SevNum; i++)
	{
		SevOn[i]=0;
	}
	
 while(AllSevOn==0)
 {
	 for (s=1; s<=SevNum;s++)  // sevro 1~10
	 {
		 SevOn[s]=RSHello485(s);
		 //sprintf(str, "RC motor(%1d) Return:  %1d \n",s, SevOn[s]); 
		 if(SevOn[s])sprintf(str, "%1d ",s);        // detect motor 
		 else sprintf(str, "%1d ",0);                 // no detect 
	   UARTEX_SendStr(1,str);ClrStr(str);
		 if(s==SevNum)UARTEX_SendStr(1," \n");
	   DelayNS(10);	 
	 }
	 
	 for(s=1; s<=SevNum; s++)
	 {
		 if(SevOn[s]==0)break;
	 }
	 if(s==(11))AllSevOn=1; // servo number +1
	 

 } 
 UARTEX_SendStr(1,"All motors be checked \n");


 
 //*************************** Arm Int *****************//
 
 while(arm_int==0)
{
// Torque on;
RSTorqueOnOff485(1, Svo1 );  
RSTorqueOnOff485(1, Svo2 );
RSTorqueOnOff485(1, Svo3 );
RSTorqueOnOff485(1, Svo4 );
RSTorqueOnOff485(1, Svo5 );
RSTorqueOnOff485(1, Svo6 );
RSTorqueOnOff485(1, Svo7 );
RSTorqueOnOff485(1, Svo8 );


//Initial Position


RSMove485( -930, 300, Svo1 ); 
RSMove485(470, 300, Svo5 );    
//DelayNS(300);
RSMove485( 364, 300, Svo2 ); 
RSMove485(-880, 300, Svo6 ); 
//DelayNS(300);
RSMove485( 128, 300, Svo3 ); 
RSMove485( 240, 300, Svo7 );
DelayNS(300);
RSMove485( -600, 300, Svo4 );   //( -22, 300, Svo4 )
RSMove485( 600, 300, Svo8 );    //( 98, 300, Svo8 )

DelayNS(500);


// Torque off;
RSTorqueOnOff485(2, Svo1 );
RSTorqueOnOff485(0, Svo2 );
RSTorqueOnOff485(2, Svo3 );
RSTorqueOnOff485(2, Svo4 );
RSTorqueOnOff485(2, Svo5 );
RSTorqueOnOff485(0, Svo6 );
RSTorqueOnOff485(2, Svo7 );
RSTorqueOnOff485(2, Svo8 );

arm_int=1;
}




//**************************** HUGGUNG*******************************************//

  HURT1_ON;    //pin =0
 
RSTorqueOnOff485(0, Svo1 );
RSTorqueOnOff485(0, Svo2 );
RSTorqueOnOff485(0, Svo3 );
RSTorqueOnOff485(0, Svo4 );


 
 while(Arm_FlagR==0)       //Arm_FlagR==0|| Arm_FlagL==0
 {
 
	
	 
	if(Arm_FlagR==0)
	{
		SAng6=RSGetAngle485(6);
		DelayNS(30);
		
		
		DelayNS(1);	


	 if(SAng6>-650)
	   {  
			  RSTorqueOnOff485(1, Svo5 ); 
		 	  RSTorqueOnOff485(1, Svo8 ); 
	      RSMove485( 0, 200, Svo5 );               //  RSMove485( -600, 200, Svo5 );
		 	  UARTEX_SendStr(1," Right arm hugging stept 1");
			
			 
		

			 
			  if(SAng6>-500)
				{
					 RSTorqueOnOff485(1, Svo5 ); 
		       RSTorqueOnOff485(1, Svo7 ); 
					 if(Arm_FlagL)
					 {
					 RSMove485( -1150, 250, Svo5 );             // RSMove485( -1400, 250, Svo5 ); 
					 UARTEX_SendStr(1,"Right  arm finish step 2");
					 DelayNS(250);
					 
					 
           }

						else
					 { 
					 RSMove485( -1150, 250, Svo5 ); 
					 UARTEX_SendStr(1,"Left arm not yet");
					 DelayNS(250);	
		 		   } 
		 
		       RSMove485( 1150, 300, Svo7 ); 
           DelayNS(150);

           RSTorqueOnOff485(1, Svo6 ); 
           RSTorqueOnOff485(1, Svo8 );
           RSMove485( -1250,300, Svo6 );   //-1200
		       DelayNS(200);
		       RSMove485( 750, 300, Svo8 );   //500
           DelayNS(500);
					 
					 //***//
					 RSTorqueOnOff485(1, Svo5 );
					 DelayNS(200);
					 
					 
					 
           UARTEX_SendStr(1,"Right  ARM finish Hugging");
		       RSTorqueOnOff485(0, Svo5 );   //torque off 
					 RSTorqueOnOff485(1, Svo6 ); 
					 RSTorqueOnOff485(1, Svo7 ); 
					 RSTorqueOnOff485(1, Svo8 ); 
					 Arm_FlagR=1;			 
    		}			
  	 }				
		
		
   }	 
	else
	{
      // skip out 
	}
	
	
	



//**************************************  HURT ***********************************************************//

 
DelayNS(1);
Speed1=RSGetSpeed485(1);
DelayNS(1);
Speed2=RSGetSpeed485(2);
DelayNS(1);
Speed3=RSGetSpeed485(3);
DelayNS(1);
Speed4=RSGetSpeed485(4);
DelayNS(1);	 
	 
	 

//sprintf(str, "Speed5=%1d  \t ",Speed5);
//UARTEX_SendStr(1,str); 
//	sprintf(str, "Speed6=%1d  \t ",Speed6);
//	UARTEX_SendStr(1,str); 
	

if( abs(Speed1)>60 || abs(Speed2)>60  || abs(Speed3)>60 || abs(Speed4)>60 )    //low 20  mid 40  hight 80  *****
{	
	
	
  UARTEX_SendStr(1,"feeling hurt");

	
	HURT1_OFF;
	delaytime= 30;
	
	sprintf(str, "SW1 %4d ", DA1);
  UARTEX_SendStr(1,str);
	// DelayNS(100);

	
}		
else
{	
	delaytime = delaytime-1;
	
}	



if(delaytime>0)
{
//UARTEX_SendStr(1,"stilsssllll hurt");
	HURT1_OFF;
	
}
else
{
  HURT1_ON;
 // UARTEX_SendStr(1,"NOOOOOOOOO hurt");
}


}      //    while(Arm_FlagR==0|| Arm_FlagL==0)      LOOP END
 


//****************************** Keep Current Position*****************************************************************//


DelayNS(500);

RSTorqueOnOff485(0, Svo5 ); 
RSTorqueOnOff485(2, Svo6 ); 
RSTorqueOnOff485(2, Svo7 ); 
RSTorqueOnOff485(2, Svo8 ); 

LED1_ON;    //Step 1 Embrace finish



//**************************************************************//

  Write_SmtMotor( 3, " UCI ", 3);   // position limitation 
  Write_SmtMotor( 3, " UDI ",3);   // position limitation 
  Write_SmtMotor( 3, " ZS  ", 3);

	
  Write_SmtMotor( 4, " UCI ", 4);   // position limitation 
  Write_SmtMotor( 4, " UDI ",4);   // position limitation 
  Write_SmtMotor( 4, " ZS  ", 4);

  Write_SmtMotor( 3, "BRKRLS", 3);   // position limitation 
//  Write_SmtMotor( 3, " G  ", 3);
  Write_SmtMotor( 4, "BRKRLS", 4);   // position limitation 
//  Write_SmtMotor( 4, " G  ", 4);





//************************************ STAND ********************************************************//



UARTEX_SendStr(1," start to stand");
 
 HURT1_ON;
 

 

while(Sd_Flag==0)
{
	

	
	// ---
	DA2=SWD2;
  sprintf(str, "SW2 %4d ", DA2);
  UARTEX_SendStr(1,str);
	UARTEX_SendStr(1,"\n");
// ---
	
		
	KAngL=AD0_data(3);
	KAngL=180-(KAngL-950)/KneeADC;
	sprintf(str, "Left Knee= %4d ", KAngL);
  UARTEX_SendStr(1,str);
	
	
	KAngR=AD0_data(2);
	KAngR=180-(1705-KAngR)/KneeADC;
	sprintf(str, "Right Knee= %4d ", KAngR);
	UARTEX_SendStr(1,str);
	UARTEX_SendStr(1,"\n");
	DelayNS(5);
	
	
	

	if(KAngL>165 &KAngR>165)
	{
    
		 BRAKE2_ON;       //Left Knee
		 DelayNS(5);
	   BRAKE1_ON;      //Right Knee
		 DelayNS(5);
		 LED3_ON;
		 KL_Flag=1;
		 KR_Flag=1;
		 Sd_Flag=1;
	}
	else
	{
		LED3_OFF;
		KR_Flag=0;
		KL_Flag=0;
	}
	
	
	if(SWD2==0)
	{
	Sd_Flag=1;
	UARTEX_SendStr(1,"Already sit downnnn command   \n" );	
	}	
	

//*****************hurt*******************************

Speed1=RSGetSpeed485(1);
DelayNS(1);
Speed2=RSGetSpeed485(2);
DelayNS(1);
Speed3=RSGetSpeed485(3);
DelayNS(1);
Speed4=RSGetSpeed485(4);
DelayNS(1);	 

	
	if( abs(Speed1)>60 || abs(Speed2)>60  || abs(Speed3)>60  ||abs(Speed4)>60 )    //low 20  mid 40  hight 80  *****
{	
	
	
  UARTEX_SendStr(1,"feeling hurt");

	
	HURT1_OFF;
	 delaytime= 30;
	
	 sprintf(str, "SW1 %4d ", DA1);
   UARTEX_SendStr(1,str);
	// DelayNS(100);

	
}		
else
{	
	delaytime = delaytime-1;
	
}	



if(delaytime>0)
{
//UARTEX_SendStr(1,"stilsssllll hurt");
	HURT1_OFF;
	
}
else
{
  HURT1_ON;
 // UARTEX_SendStr(1,"NOOOOOOOOO hurt");
}

//************************************************
	
	  
}		


LED1_ON;   //Stand Finish
UARTEX_SendStr(1,"Stand finish!! Waiting for sitting command! \n" );




RSTorqueOnOff485(0, Svo1 ); 
RSTorqueOnOff485(2, Svo2 ); 
RSTorqueOnOff485(2, Svo3 ); 
RSTorqueOnOff485(2, Svo4 ); 


RSTorqueOnOff485(0, Svo5 ); 
RSTorqueOnOff485(2, Svo6 ); 
RSTorqueOnOff485(2, Svo7 ); 
RSTorqueOnOff485(2, Svo8 ); 



//*****************************************SITTING *****************************************************//



while(Sit_Flag==0)  //Wait for Sit down Command
{
  

	DA2=SWD2;
  sprintf(str, "SW2 %4d ", DA2);
  UARTEX_SendStr(1,str);
	UARTEX_SendStr(1,"\n");

	
 KAngL=AD0_data(3);
KAngL=180-(KAngL-950)/KneeADC;
sprintf(str, "Left Knee= %4d ", KAngL);
 UARTEX_SendStr(1,str);
	
	
KAngR=AD0_data(2);
KAngR=180-(1705-KAngR)/KneeADC;
sprintf(str, "Right Knee= %4d ", KAngR);
UARTEX_SendStr(1,str);
	
UARTEX_SendStr(1,"\n");
 DelayNS(5);	
	
	
if(SWD2==0)   //Receive Sit Down Command
	{ 
		
		LED1_OFF;
		LED2_OFF;
		LED3_OFF;
		BRAKE1_OFF;   // knee unlock
		BRAKE2_OFF;
		
	}

	
	
if(KAngR<110||KAngL<110)
{	
	
  UARTEX_SendStr(1,"Sit finish!! I will release all of the arm \n" );
	
  RSTorqueOnOff485(0, 1 );        //torque off 
	RSTorqueOnOff485(0, 2 );
  RSTorqueOnOff485(0, 3 );
	RSTorqueOnOff485(0, 4 );
	RSTorqueOnOff485(0, 5 );
	RSTorqueOnOff485(0, 6 );
  RSTorqueOnOff485(0, 7 );
	RSTorqueOnOff485(0, 8 );
	RSTorqueOnOff485(0, 9 );
	RSTorqueOnOff485(0, 10 );	
	//Sit_Flag=1;                             //sit flag 
	DelayNS(5);

}

 else
 {
 DelayNS(5);
 }

 
 
 
 
//************** hurt ******************//
	

Speed1=RSGetSpeed485(1);
DelayNS(1);
Speed2=RSGetSpeed485(2);
DelayNS(1);
Speed3=RSGetSpeed485(3);
DelayNS(1);
Speed4=RSGetSpeed485(4);
DelayNS(1);	 	
	
	
if( abs(Speed1)>60 || abs(Speed2)>60  || abs(Speed3)>60  ||abs(Speed4)>60 )    //low 20  mid 40  hight 80  *****
{	
	
	
  UARTEX_SendStr(1,"feeling hurt");

	
	HURT1_OFF;
	 delaytime = 1;                                 //  time delay  slow 1  mid 25  long  45   *****
	
	 sprintf(str, "SW1 %4d ", DA1);
   UARTEX_SendStr(1,str);
	// DelayNS(100);

	
}		
else
{	
	delaytime =delaytime-1;
	
}	



if(delaytime>0)
{
//UARTEX_SendStr(1,"stilsssllll hurt");
	HURT1_OFF;
	
}
else
{
  HURT1_ON;
 // UARTEX_SendStr(1,"NOOOOOOOOO hurt");
}
 
//*********hurt end********//
 
}




    return 0;
}
/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/








