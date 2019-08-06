
/*
---------------------------------------------------------------------------------------------------------
*********************************************************************************************************
** 文件名称 ：	I2CINT.c
** 功能说明 ：	LPC2000硬件I2C中断方式软件包。
** 使用说明 ：  主程序要配置好I2C总线接口(I2C引脚功能和I2C中断，并已使能I2C主模式)
*********************************************************************************************************
---------------------------------------------------------------------------------------------------------
*/
#include  "config.h" 

/******************************************************************************/
/*SC16IS752 寄存器组*/
/******************************************************************************/
/* 通用寄存器 */   
#define SC16IS752_THR       0x00             /* 发送保存寄存器 */    /*W*/
#define SC16IS752_RHR       0x00             /* 接收保存寄存器 */    /*R*/
#define SC16IS752_IER       0x01             /* 中断使能寄存器 */    /*R/W*/
#define SC16IS752_FCR       0x02             /* FIFO控制寄存器 */    /*W*/
#define SC16IS752_IIR       0x02             /* 中断识别寄存器*/     /*R*/
#define SC16IS752_LCR       0x03             /* 线控制寄存器 */      /*R/W*/
   
#define SC16IS752_S_MCR     0x04             /* MODEM控制寄存器 */   /*R/W*/ 
#define SC16IS752_LSR       0x05             /* 线状态寄存器*/       /*R*/
#define SC16IS752_MSR       0x06             /* MODEM状态寄存器 */   /*R*/ 
#define SC16IS752_SPR       0x07             /* 暂存寄存器 */        /*R/W*/
   
#define SC16IS752_TCR       0x06             /* 发送控制寄存器 */    /*R/W*/
#define SC16IS752_TLR       0x07             /* 触发点寄存器 */      /*R/W*/
#define SC16IS752_TXLVL     0x08             /* 发送FIFO电平寄存器 *//*R*/    
#define SC16IS752_RXLVL     0x09             /* 接收FIFO电平寄存器 *//*R*/    
   
#define SC16IS752_IODir     0x0A             /* I/O脚方向寄存器 */   /*R/W*/ 
#define SC16IS752_IOState   0x0B             /* I/O脚状态寄存器*/    /*R/W*/
#define SC16IS752_IOIntEna  0x0C             /* I/O中断使能寄存器 */ /*R/W*/   
#define SC16IS752_IOControl 0x0E             /* I/O脚控制寄存器 */   /*R/W*/ 
   
#define SC16IS752_EFCR      0x0F             /* 额外特性寄存器 */    /*R/W*/
   
/*特殊寄存器 */   
#define SC16IS752_DLL       0x00             /* 除数锁存器LSB */     /*R/W*/
#define SC16IS752_DLH       0x01             /* 除数锁存器MSB */     /*R/W*/
   
/*增强型寄存器 */   
#define SC16IS752_EFR       0x02             /* 增强型特性寄存器 */  /*R/W*/  
#define SC16IS752_Xon1      0x04             /* Xon1字 */            /*R/W*/
#define SC16IS752_Xon2      0x05             /* Xon2字*/             /*R/W*/
#define SC16IS752_Xoff1     0x06             /* Xoff1字*/            /*R/W*/
#define SC16IS752_Xoff2     0x07             /* Xoff2字*/            /*R/W*/
#define REC_TIME_DEL        0x02             //接收超时
#define SED_TIME_DEL        0x02             //发送超时   
#define SC16IS752_FIFO_NUM     64  /* FIFO长度 */ 

extern  void MSpi1Int(uint8 fdiv)	;
extern  void SPI_wr_752 (uint8 reg, uint8 data, uint8 channel);
extern  uint8 SPI_rd_752 (uint8 reg, uint8 channel);
extern  void init_mcu_port (void);
extern  void SC16IS752_Init_ChA1_232 (void);
extern  void SC16IS752_Init_ChB1_232 (void);
extern  void SC16IS752_Init_ChA2_232 (void);
extern  void SC16IS752_Init_ChB2_232 (void);
extern  void SC16IS752_Init_ChA3_232 (void);
extern  void SC16IS752_Init_ChB3_232 (void);
extern  void SC16IS752_Init_ChA1_GM (void);
extern  void SC16IS752_Init_ChB1_GM (void);
extern  void SC16IS752_Init_ChA2_GM (void);
extern  void SC16IS752_Init_ChB2_GM (void);
extern  void SC16IS752_Init_ChA3_GM (void);
extern  void SC16IS752_Init_ChB3_GM (void);
extern  void Set_GPIO_Dir(uint8 channel,uint8 logic);
extern  uint8 Read_GPIO(uint8 channel);
extern  void Write_GPIO(uint8 channel,uint8 data);   

#define Adr752 0x90

#define MSTR (1<< 5)
#define CPOL (1<< 4)
#define CPHA (1<< 3)
#define LSBF (1<< 6)
#define SPI_MODE (MSTR)


volatile  uint8  I2CCommand[2];



/****************************************************************I2c 相关函数***********************************/

/* 定义用于和I2C中断传递信息的全局变量 */
volatile uint8 	I2C_sla;			/* I2C器件从地址 					*/
volatile uint32	I2C_suba;			/* I2C器件内部子地址 				*/
volatile uint8 	I2C_suba_num;		/* I2C子地址字节数					*/
volatile uint8 	*I2C_buf;        	/* 数据缓冲区指针  					*/
volatile uint32 I2C_num;			/* 要读取/写入的数据个数 			*/
volatile uint8 	I2C_end;			/* I2C总线结束标志：结束总线是置1 	*/
volatile uint8 	I2C_suba_en;		/* 	子地址控制。
										0--子地址已经处理或者不需要子地址
										1--读取操作
										2--写操作
									*/


/*
*********************************************************************************************************
** 函数名称 ：I2cInit()
** 函数功能 ：I2C初始化
** 入口参数 ：Fi2c	I2C总线频率(最大400K)
** 出口参数 ：无
*********************************************************************************************************
*/

void I2cInit(uint32 Fi2c)
{
	if (Fi2c > 400000)
		Fi2c = 400000;
		
   	//PINSEL0 = (PINSEL0 & 0xFFFFFF0F) | 0x50; 			/* 设置I2C控制口有效 				*/
	PINSEL0 = (PINSEL0 & (~0xF0)) | 0x50; 	// 不影响其它管脚连接
	I2SCLH = (Fpclk/Fi2c + 1) / 2;						/* 设定I2C时钟 						*/
	I2SCLL = (Fpclk/Fi2c)/2;
	I2CONCLR = 0x2C;
	I2CONSET = 0x40;									/* 使能主I2C 						*/
	
	/* 设置I2C中断允许 */
	VICIntSelect = 0x00000000;							/* 设置所有通道为IRQ中断 			*/
	VICVectCntl0 = (0x20 | 0x09);						/* I2C通道分配到IRQ slot0，最高优先级 */
	VICVectAddr0 = (int32)IRQ_I2C;						/* 设置I2C中断向量 					*/
	VICIntEnable = (0x01 << 9);							/* 使能I2C中断 						*/
}

/*
**********************************************************************************************************
** 函数名称：ISendByte()
** 函数功能：向无子地址器件发送1字节数据。
** 入口参数：sla		器件地址
**           dat		要发送的数据
** 出口参数：返回值为0时表示出错，为1时表示操作正确。
** 说明：	使用前要初始化好I2C引脚功能和I2C中断，并已使能I2C主模式
*********************************************************************************************************
*/
uint8  ISendByte(uint8 sla, uint8 dat)
{  /* 参数设置 */
   I2C_sla     = sla;		// 写操作的器件地址
   I2C_buf     = &dat;		// 待发送的数据
   I2C_num     = 1;			// 发送1字节数据
   I2C_suba_en = 0;		 	// 无子地址
   I2C_end     = 0;
   
   I2CONCLR = 0x2C;
   I2CONSET = 0x60;             // 设置为主机，并启动总线
   
   while(0==I2C_end);
   if(1==I2C_end) return(1);
     else return(0);
}

/*
*********************************************************************************************************
** 函数名称：IRcvByte()
** 函数功能：向无子地址器件读取1字节数据。
** 入口参数：sla		器件地址
**           dat		接收数据的变量指针
** 出口参数：返回值为0时表示操作出错，为1时表示操作正确。
** 说明：使用前要初始化好I2C引脚功能和I2C中断，并已使能I2C主模式
*********************************************************************************************************
*/
uint8  IRcvByte(uint8 sla, uint8 *dat)
{  /* 参数设置 */
   I2C_sla     = sla+1;		// 读操作的器件地址
   I2C_buf     = dat;
   I2C_num     = 1;
   I2C_suba_en = 0;			// 无子地址
   I2C_end     = 0;
   
   I2CONCLR = 0x2C;
   I2CONSET = 0x60;         // 设置为主机，并启动总线
   
   while(0==I2C_end);
   if(1==I2C_end) return(1);
     else return(0);
}


/*
*********************************************************************************************************
** 函数名称 ：I2C_ReadNByte()
** 函数功能 ：从有子地址器件任意地址开始读取N字节数据
** 入口参数 ：	sla			器件从地址
**				suba_type	子地址结构	1－单字节地址	2－8+X结构	2－双字节地址
**				suba		器件子地址
**				s			数据接收缓冲区指针
**				num			读取的个数
** 出口参数 ：	TRUE		操作成功
**				FALSE		操作失败
*********************************************************************************************************
*/
uint8 I2C_ReadNByte (uint8 sla, uint32 suba_type, uint32 suba, uint8 *s, uint32 num)
{
	if (num > 0)	/* 判断num个数的合法性 */
	{	/* 参数设置 */
		if (suba_type == 1)
		{	/* 子地址为单字节 */
			I2C_sla     	= sla + 1;							/* 读器件的从地址，R=1 	*/
			I2C_suba    	= suba;								/* 器件子地址 			*/
			I2C_suba_num	= 1;								/* 器件子地址为1字节 	*/
		}
		if (suba_type == 2)
		{	/* 子地址为2字节 */
			I2C_sla     	= sla + 1;							/* 读器件的从地址，R=1 	*/
			I2C_suba   	 	= suba;								/* 器件子地址 			*/
			I2C_suba_num	= 2;								/* 器件子地址为2字节 	*/
		}
		if (suba_type == 3)
		{	/* 子地址结构为8+X*/
			I2C_sla			= sla + ((suba >> 7 )& 0x0e) + 1;	/* 读器件的从地址，R=1	*/
			I2C_suba		= suba & 0x0ff;						/* 器件子地址	 		*/
			I2C_suba_num	= 1;								/* 器件子地址为8+x	 	*/
		}
		I2C_buf     = s;										/* 数据接收缓冲区指针 	*/
		I2C_num     = num;										/* 要读取的个数 		*/
		I2C_suba_en = 1;										/* 有子地址读 			*/
		I2C_end     = 0;
		
		/* 清除STA,SI,AA标志位 */
		I2CONCLR = 	(1 << 2)|	/* AA 		*/
					(1 << 3)|	/* SI 		*/
					(1 << 5);	/* STA 		*/
		
		/* 置位STA,启动I2C总线 */
		I2CONSET = 	(1 << 5)|	/* STA 		*/
					(1 << 6);	/* I2CEN 	*/
		
		/* 等待I2C操作完成 */
		while (I2C_end == 0)
		{	}
		if (I2C_end == 1)
			return (TRUE);
		else
			return (FALSE);			
	}
	return (FALSE);
}

/*
*********************************************************************************************************
** 函数名称 ：I2C_WriteNByte()
** 函数功能 ：向有子地址器件写入N字节数据
** 入口参数 ：	sla			器件从地址
**				suba_type	子地址结构	1－单字节地址	3－8+X结构	2－双字节地址
**			  	suba		器件内部物理地址
**			  	*s			将要写入的数据的指针
**			  	num			将要写入的数据的个数
** 出口参数 ：	TRUE		操作成功
**			  	FALSE		操作失败
*********************************************************************************************************
*/
uint8 I2C_WriteNByte(uint8 sla, uint8 suba_type, uint32 suba, uint8 *s, uint32 num)
{
	if (num > 0)/* 如果读取的个数为0，则返回错误 */
	{	/* 设置参数 */	
		if (suba_type == 1)
		{	/* 子地址为单字节 */
			I2C_sla     	= sla;								/* 读器件的从地址	 	*/
			I2C_suba    	= suba;								/* 器件子地址 			*/
			I2C_suba_num	= 1;								/* 器件子地址为1字节 	*/
		}
		if (suba_type == 2)
		{	/* 子地址为2字节 */
			I2C_sla     	= sla;								/* 读器件的从地址 		*/
			I2C_suba   	 	= suba;								/* 器件子地址 			*/
			I2C_suba_num	= 2;								/* 器件子地址为2字节 	*/
		}
		if (suba_type == 3)
		{	/* 子地址结构为8+X */
			I2C_sla			= sla + ((suba >> 7 )& 0x0e);		/* 读器件的从地址		*/
			I2C_suba		= suba & 0x0ff;						/* 器件子地址			*/
			I2C_suba_num	= 1;								/* 器件子地址为8+X	 	*/
		}

		I2C_buf     = s;										/* 数据 				*/
		I2C_num     = num;										/* 数据个数 			*/
		I2C_suba_en = 2;										/* 有子地址，写操作 	*/
		I2C_end     = 0;
		
		/* 清除STA,SI,AA标志位 */
		I2CONCLR = 	(1 << 2)|	/* AA 	*/
					(1 << 3)|	/* SI 	*/
					(1 << 5);	/* STA 	*/
		
		/* 置位STA,启动I2C总线 */
		I2CONSET = 	(1 << 5)|	/* STA 	*/
					(1 << 6);	/* I2CEN*/
		
		/* 等待I2C操作完成 */
		while (I2C_end == 0) 
		{	}
		if (I2C_end == 1)
			return (TRUE);
		else
			return (FALSE);	
	}
	return (FALSE);
}

/*
*********************************************************************************************************
** 函数名称 ：__irq IRQ_I2C()
** 函数名次 ：硬件I2C中断服务程序。 
** 入口参数 ：无
** 出口参数 ：无
** 说明     ：注意处理子地址为2字节的情况。 
*********************************************************************************************************
*/
void __irq IRQ_I2C(void)
{	/* 读取I2C状态寄存器I2DAT */
	/* 按照全局变量的设置进行操作及设置软件标志 */
	/* 清除中断逻辑,中断返回 */
	
	switch (I2STAT & 0xF8)
	{	/* 根据状态码进行相应的处理 */
		case 0x08:	/* 已发送起始条件 */				/* 主发送和主接收都有 		*/
			/* 装入SLA+W或者SLA+R */
		 	if(I2C_suba_en == 1)/* SLA+R */				/* 指定子地址读 			*/
		 	{	I2DAT = I2C_sla & 0xFE; 				/* 先写入地址 				*/
		 	}
            else	/* SLA+W */
            {  	I2DAT = I2C_sla;        				/* 否则直接发送从机地址 	*/
            }
            /* 清零SI位 */
            I2CONCLR =	(1 << 3)|						/* SI 						*/
            			(1 << 5);						/* STA 						*/
            break;
            
       	case 0x10:	/*已发送重复起始条件 */ 			/* 主发送和主接收都有 		*/
       		/* 装入SLA+W或者SLA+R */
       		I2DAT = I2C_sla;							/* 重起总线后，重发从地址 	*/
       		I2CONCLR = 0x28;							/* 清零SI,STA */
       		break;

		case 0x18:
       	case 0x28:	/* 已发送I2DAT中的数据，已接收ACK */
       		if (I2C_suba_en == 0)
       		{
	       		if (I2C_num > 0)
	       		{	I2DAT = *I2C_buf++;
	       			I2CONCLR = 0x28;					/* 清零SI,STA 				*/
	       			I2C_num--;
	       		}
	       		else	/* 没有数据发送了 */
	       		{		/* 停止总线 */
	       		  	I2CONSET = (1 << 4);				/* STO 						*/
	       			I2CONCLR = 0x28;					/* 清零SI,STA 				*/
	       		  	I2C_end = 1;						/* 总线已经停止 			*/
	       		}
       		}
       		
            if(I2C_suba_en == 1)	/* 若是指定地址读，则重新启动总线 				*/
            { 
            	if (I2C_suba_num == 2)
            	{	I2DAT = ((I2C_suba >> 8) & 0xff);
	       			I2CONCLR = 0x28;					/* 清零SI,STA 				*/
	       			I2C_suba_num--;
	       			break;	
	       		} 
	       		
	       		if(I2C_suba_num == 1)
	       		{	I2DAT = (I2C_suba & 0xff);
	       			I2CONCLR = 0x28;					/* 清零SI,STA 				*/
	       			I2C_suba_num--;
	       			break;	
	       		}
	       		
            	if (I2C_suba_num == 0)
            	{	I2CONSET = 0x20;
               		I2CONCLR = 0x08;
               		I2C_suba_en = 0;     				/* 子地址己处理 			*/
               		break;
               	}
            }
            
            if (I2C_suba_en == 2)/* 指定子地址写,子地址尚未指定,则发送子地址 		*/
       		{
       		 	if (I2C_suba_num > 0)
            	{	if (I2C_suba_num == 2)
            		{	I2DAT = ((I2C_suba >> 8) & 0xff);
            			I2CONCLR = 0x28;
            			I2C_suba_num--;
            			break;
            		}
            		if (I2C_suba_num == 1)
            		{	I2DAT    = (I2C_suba & 0xff);
               			I2CONCLR = 0x28;
               			I2C_suba_num--;
               			I2C_suba_en  = 0;
               			break;
               		}
               	}
             }
       		break;
       		  
       case 0x40:	/* 已发送SLA+R,已接收ACK */
       		if (I2C_num <= 1)	/* 如果是最后一个字节 */			
       		{	I2CONCLR = 1 << 2;      				/* 下次发送非应答信号 		*/
       		}
       		else
       		{ 	I2CONSET = 1 << 2;						/* 下次发送应答信号 		*/
       		}
       		I2CONCLR = 0x28;							/* 清零SI,STA 				*/
       		break;

       	case 0x20:	/* 已发送SLA+W,已接收非应答              */
       	case 0x30:	/* 已发送I2DAT中的数据，已接收非应答     */
       	case 0x38:	/* 在SLA+R/W或数据字节中丢失仲裁         */
   		case 0x48:	/* 已发送SLA+R,已接收非应答              */
         	I2CONCLR = 0x28;
            I2C_end = 0xFF; 
       		break;   				
	
		case 0x50:	/* 已接收数据字节，已返回ACK */
			*I2C_buf++ = I2DAT;
			I2C_num--;
			if (I2C_num == 1)/* 接收最后一个字节 */
			{  	I2CONCLR = 0x2C;						/* STA,SI,AA = 0 			*/
			}
			else
			{  	I2CONSET = 0x04;						/* AA=1 					*/
			  	I2CONCLR = 0x28;
			}
			break;
		
		case 0x58:	/* 已接收数据字节，已返回非应答 */
			*I2C_buf++ = I2DAT;     					/* 读取最后一字节数据 		*/
            I2CONSET = 0x10;        					/* 结束总线 				*/
            I2CONCLR = 0x28;
            I2C_end = 1; 
            break;
            
      	default:
      		break;
	}
   VICVectAddr = 0x00;              					/* 中断处理结束 			*/
}


/*******************************************SC16IS752函数***********************************************************************************/

/****************************************************************************
* SPI_Wr_752(uint8 reg, uint8 data, uint8 channel)
* Function: Write SC16IS762
* Input： uint8 reg, uint8 data, uint8 slaver
* output: none
****************************************************************************/
void SPI_wr_752(uint8 reg, uint8 data, uint8 channel)
{
	char str[2];
  reg <<= 3; // 寄存器地址

  if ((channel==0x01)) 
   reg |= 0x02; // 通道地址
  
	str[0]=data;
  I2C_WriteNByte(Adr752, 1, reg, str, 1);

}

/****************************************************************************
* uint8 SPI_rd_752(uint8 reg, uint8 channel)
* Function: Read SC16IS762
* Input： uint8 reg, uint8 channel
* output: data
****************************************************************************/
uint8 SPI_rd_752 (uint8 reg, uint8 channel)

{ 
  uint8 rev[2];
  reg = (reg<<3) | 0x80; // register address byte
  if ((channel==0x01)) 
     reg |= 0x02; // channel address byte
     //SPICommand[0] = reg; // register address 
  
	I2C_ReadNByte (Adr752, 1, reg, rev, 1);
  //IRcvByte(reg, &temp1);
	
  //SPI1_SendByte(SPICommand[0]);
  //temp1 = SPI1_RecByte();
  

  return(rev[0]); 
} 

/******************************************************************************/
/* 函数名称：Set_GPIO_Dir();                               */
/* 功能描述：SC16IS752_IODir  IO方向寄存器  [7:0]  0输入 1输出 */
/*			 SC16IS752_IOControl bit0 IO锁存 bit1 gpio[7:4]/modem bit2 gpio[3:0]/modem 0 io 1modem */
/* 参数说明：logic   管脚输入还是输出 */
/*              channel   所属通道                          */
/* 参数返回：无 */
/******************************************************************************/
void Set_GPIO_Dir(uint8 channel,uint8 logic)                                                                  
{ // Set Direction on UART GPIO Port pins GPIO0 to GPIO7 
     // 0=input   1=Output  
  SPI_wr_752 (SC16IS752_LCR,SPI_rd_752(SC16IS752_LCR,channel)&0x7F, channel);
	SPI_wr_752 (SC16IS752_IOControl, 0x03,channel); // Set the IOControl Register to GPIO Control 
	SPI_wr_752 (SC16IS752_IODir,logic,channel); // output the control bits to the IO Direction Register 
} 
/******************************************************************************/
/* 函数名称：Read_GPIO();                               */
/* 功能描述：读管脚状态值      读返回管脚状态                       */
/* 参数说明：channel   所属通道	*/
/* 参数返回：读出数据 */
  
/******************************************************************************/
uint8 Read_GPIO(uint8 channel)                                                                  
   { // Read UART GPIO Port 
	uint8 data = 0x00;  
	SPI_wr_752 (SC16IS752_LCR,SPI_rd_752(SC16IS752_LCR,channel)&0x7F, channel);
	data=SPI_rd_752(SC16IS752_IOState,channel); // get GPIO Bits state 0-7 

// return data bits state or zero 
	return(data); 
    } 
/******************************************************************************/
/* 函数名称：Write_GPIO();                               */
/* 功能描述：写管脚状态值     写SC16IS752_IOState 0输出0 1输出1                       */
/* 参数说明：reg   寄存器地址
              channel   所属通道                          */
/* 参数返回：读出数据 */
  
/******************************************************************************/
void Write_GPIO(uint8 channel,uint8 data)                                                                  
	{ // Load UART GPIO Port
	SPI_wr_752 (SC16IS752_LCR,SPI_rd_752(SC16IS752_LCR,channel)&0x7F, channel);        
	SPI_wr_752	(SC16IS752_IOState,data, channel); // set GPIO Output pins state 0-7 
	} 

void SetLED(uint8 channel, uint8 on)
{
		char IOdat;
	  IOdat=Read_GPIO(0);
    if(on)IOdat=IOdat& ~(0x01<<channel);
	  else IOdat=( IOdat& ~(0x01<<channel))|(0x01<<channel);

    Set_GPIO_Dir(0,0xff); //GPIO0 to GPIO7 Set as Output 
    Write_GPIO(0,IOdat);	

}


/******************************************************************************/
/* 函数名称：SC16IS752_Init_ChA1_232();                               */
/* 功能描述：初始化SC16IS752端口a                           */
/* 参数说明：无                          */
/* 参数返回：无 */
  
/******************************************************************************/
void SC16IS752_Init_ChA1_232 (void) 

{

SPI_wr_752 (SC16IS752_LCR, 0x80, 0);
SPI_wr_752 (SC16IS752_DLL, 1, 0);  //115200
SPI_wr_752 (SC16IS752_DLH, 0x00, 0);  
SPI_wr_752 (SC16IS752_LCR, 0xBF, 0); //1011 1111 
SPI_wr_752 (SC16IS752_EFR, 0x10, 0);
SPI_wr_752 (SC16IS752_LCR, 0x03, 0);
SPI_wr_752 (SC16IS752_FCR, 0x07, 0); //接收、发送触发均为8字节
SPI_wr_752 (SC16IS752_SPR, 'A', 0);

SPI_wr_752 (SC16IS752_IER, 0x01, 0); 

} 
/******************************************************************************/
/* 函数名称：SC16IS752_Init_ChB1_232();                               */
/* 功能描述：初始化SC16IS752端口b                           */
/* 参数说明：无                          */
/* 参数返回：无 */
  
/******************************************************************************/
void SC16IS752_Init_ChB1_232 (void)

{ 

SPI_wr_752 (SC16IS752_LCR, 0x80, 1);
SPI_wr_752 (SC16IS752_DLL, 1, 1);  //115200
SPI_wr_752 (SC16IS752_DLH, 0x00, 1);  
SPI_wr_752 (SC16IS752_LCR, 0xBF, 1); 
SPI_wr_752 (SC16IS752_EFR, 0x10, 1);
SPI_wr_752 (SC16IS752_LCR, 0x03, 1);
SPI_wr_752 (SC16IS752_FCR, 0x07, 1);  
SPI_wr_752 (SC16IS752_SPR, 'B', 1);

SPI_wr_752 (SC16IS752_IER, 0x01, 1); 
} 


void  UARTEX_SendByte(uint8 channel, uint8 data)
{
	
	 SPI_wr_752 (SC16IS752_THR,data,channel);	    // Send data
	 while(  (SPI_rd_752(SC16IS752_LSR,channel)& (0x01<<5)) ==0);
}


void  UARTEX_SendStr(uint8 channel, uint8  *str)
{  
	/* 
	switch(channel)
	 {
		 case 0:
			 SC16IS752_Init_ChA1_232 (); 
			 break;
		 case 1:
			 SC16IS752_Init_ChB1_232 (); 
			 break;
		
		 default:
			 break;
	 }
	 */
	 SetLED(channel,1);
	 while(1)
   {  if( *str == '\0' ) break; 
      UARTEX_SendByte( channel, *str++);   //send data
		  //SPI_wr_752 (SC16IS752_THR,*str++,channel);	    // Send data
   }
	 SetLED(channel,0);
}


//Receive Byte

uint8  UARTEX_RevByte(uint8 channel, uint8* str)
{
	 int i;
	 i=0;
	 //SPI_wr_752 (SC16IS752_THR,data,channel);	    // Send data
	 while(  (SPI_rd_752(SC16IS752_LSR,channel)& (0x01)) ==1)
	 { 
		 str[i++]=SPI_rd_752(SC16IS752_RHR,channel);
	 }
	 if(i>2)i-=2;
	 return(i);
 	 
}


//Receive Data
void  UARTEX_RevStr(uint8 channel, uint8  *str, uint8 n)
{  
	/* 
	switch(channel)
	 {
		 case 0:
			 SC16IS752_Init_ChA1_232 (); 
			 break;
		 case 1:
			 SC16IS752_Init_ChB1_232 (); 
			 break;
		
		 default:
			 break;
	 }
	 */
	 SetLED(channel,1);
	 while(1)
   {  if( *str == '\0' ) break; 
      UARTEX_SendByte( channel, *str++);   //send data
		  //SPI_wr_752 (SC16IS752_THR,*str++,channel);	    // Send data
   }
	 SetLED(channel,0);
}

	