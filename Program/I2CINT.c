
/*
---------------------------------------------------------------------------------------------------------
*********************************************************************************************************
** �ļ����� ��	I2CINT.c
** ����˵�� ��	LPC2000Ӳ��I2C�жϷ�ʽ�������
** ʹ��˵�� ��  ������Ҫ���ú�I2C���߽ӿ�(I2C���Ź��ܺ�I2C�жϣ�����ʹ��I2C��ģʽ)
*********************************************************************************************************
---------------------------------------------------------------------------------------------------------
*/
#include  "config.h" 

/******************************************************************************/
/*SC16IS752 �Ĵ�����*/
/******************************************************************************/
/* ͨ�üĴ��� */   
#define SC16IS752_THR       0x00             /* ���ͱ���Ĵ��� */    /*W*/
#define SC16IS752_RHR       0x00             /* ���ձ���Ĵ��� */    /*R*/
#define SC16IS752_IER       0x01             /* �ж�ʹ�ܼĴ��� */    /*R/W*/
#define SC16IS752_FCR       0x02             /* FIFO���ƼĴ��� */    /*W*/
#define SC16IS752_IIR       0x02             /* �ж�ʶ��Ĵ���*/     /*R*/
#define SC16IS752_LCR       0x03             /* �߿��ƼĴ��� */      /*R/W*/
   
#define SC16IS752_S_MCR     0x04             /* MODEM���ƼĴ��� */   /*R/W*/ 
#define SC16IS752_LSR       0x05             /* ��״̬�Ĵ���*/       /*R*/
#define SC16IS752_MSR       0x06             /* MODEM״̬�Ĵ��� */   /*R*/ 
#define SC16IS752_SPR       0x07             /* �ݴ�Ĵ��� */        /*R/W*/
   
#define SC16IS752_TCR       0x06             /* ���Ϳ��ƼĴ��� */    /*R/W*/
#define SC16IS752_TLR       0x07             /* ������Ĵ��� */      /*R/W*/
#define SC16IS752_TXLVL     0x08             /* ����FIFO��ƽ�Ĵ��� *//*R*/    
#define SC16IS752_RXLVL     0x09             /* ����FIFO��ƽ�Ĵ��� *//*R*/    
   
#define SC16IS752_IODir     0x0A             /* I/O�ŷ���Ĵ��� */   /*R/W*/ 
#define SC16IS752_IOState   0x0B             /* I/O��״̬�Ĵ���*/    /*R/W*/
#define SC16IS752_IOIntEna  0x0C             /* I/O�ж�ʹ�ܼĴ��� */ /*R/W*/   
#define SC16IS752_IOControl 0x0E             /* I/O�ſ��ƼĴ��� */   /*R/W*/ 
   
#define SC16IS752_EFCR      0x0F             /* �������ԼĴ��� */    /*R/W*/
   
/*����Ĵ��� */   
#define SC16IS752_DLL       0x00             /* ����������LSB */     /*R/W*/
#define SC16IS752_DLH       0x01             /* ����������MSB */     /*R/W*/
   
/*��ǿ�ͼĴ��� */   
#define SC16IS752_EFR       0x02             /* ��ǿ�����ԼĴ��� */  /*R/W*/  
#define SC16IS752_Xon1      0x04             /* Xon1�� */            /*R/W*/
#define SC16IS752_Xon2      0x05             /* Xon2��*/             /*R/W*/
#define SC16IS752_Xoff1     0x06             /* Xoff1��*/            /*R/W*/
#define SC16IS752_Xoff2     0x07             /* Xoff2��*/            /*R/W*/
#define REC_TIME_DEL        0x02             //���ճ�ʱ
#define SED_TIME_DEL        0x02             //���ͳ�ʱ   
#define SC16IS752_FIFO_NUM     64  /* FIFO���� */ 

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



/****************************************************************I2c ��غ���***********************************/

/* �������ں�I2C�жϴ�����Ϣ��ȫ�ֱ��� */
volatile uint8 	I2C_sla;			/* I2C�����ӵ�ַ 					*/
volatile uint32	I2C_suba;			/* I2C�����ڲ��ӵ�ַ 				*/
volatile uint8 	I2C_suba_num;		/* I2C�ӵ�ַ�ֽ���					*/
volatile uint8 	*I2C_buf;        	/* ���ݻ�����ָ��  					*/
volatile uint32 I2C_num;			/* Ҫ��ȡ/д������ݸ��� 			*/
volatile uint8 	I2C_end;			/* I2C���߽�����־��������������1 	*/
volatile uint8 	I2C_suba_en;		/* 	�ӵ�ַ���ơ�
										0--�ӵ�ַ�Ѿ�������߲���Ҫ�ӵ�ַ
										1--��ȡ����
										2--д����
									*/


/*
*********************************************************************************************************
** �������� ��I2cInit()
** �������� ��I2C��ʼ��
** ��ڲ��� ��Fi2c	I2C����Ƶ��(���400K)
** ���ڲ��� ����
*********************************************************************************************************
*/

void I2cInit(uint32 Fi2c)
{
	if (Fi2c > 400000)
		Fi2c = 400000;
		
   	//PINSEL0 = (PINSEL0 & 0xFFFFFF0F) | 0x50; 			/* ����I2C���ƿ���Ч 				*/
	PINSEL0 = (PINSEL0 & (~0xF0)) | 0x50; 	// ��Ӱ�������ܽ�����
	I2SCLH = (Fpclk/Fi2c + 1) / 2;						/* �趨I2Cʱ�� 						*/
	I2SCLL = (Fpclk/Fi2c)/2;
	I2CONCLR = 0x2C;
	I2CONSET = 0x40;									/* ʹ����I2C 						*/
	
	/* ����I2C�ж����� */
	VICIntSelect = 0x00000000;							/* ��������ͨ��ΪIRQ�ж� 			*/
	VICVectCntl0 = (0x20 | 0x09);						/* I2Cͨ�����䵽IRQ slot0��������ȼ� */
	VICVectAddr0 = (int32)IRQ_I2C;						/* ����I2C�ж����� 					*/
	VICIntEnable = (0x01 << 9);							/* ʹ��I2C�ж� 						*/
}

/*
**********************************************************************************************************
** �������ƣ�ISendByte()
** �������ܣ������ӵ�ַ��������1�ֽ����ݡ�
** ��ڲ�����sla		������ַ
**           dat		Ҫ���͵�����
** ���ڲ���������ֵΪ0ʱ��ʾ����Ϊ1ʱ��ʾ������ȷ��
** ˵����	ʹ��ǰҪ��ʼ����I2C���Ź��ܺ�I2C�жϣ�����ʹ��I2C��ģʽ
*********************************************************************************************************
*/
uint8  ISendByte(uint8 sla, uint8 dat)
{  /* �������� */
   I2C_sla     = sla;		// д������������ַ
   I2C_buf     = &dat;		// �����͵�����
   I2C_num     = 1;			// ����1�ֽ�����
   I2C_suba_en = 0;		 	// ���ӵ�ַ
   I2C_end     = 0;
   
   I2CONCLR = 0x2C;
   I2CONSET = 0x60;             // ����Ϊ����������������
   
   while(0==I2C_end);
   if(1==I2C_end) return(1);
     else return(0);
}

/*
*********************************************************************************************************
** �������ƣ�IRcvByte()
** �������ܣ������ӵ�ַ������ȡ1�ֽ����ݡ�
** ��ڲ�����sla		������ַ
**           dat		�������ݵı���ָ��
** ���ڲ���������ֵΪ0ʱ��ʾ��������Ϊ1ʱ��ʾ������ȷ��
** ˵����ʹ��ǰҪ��ʼ����I2C���Ź��ܺ�I2C�жϣ�����ʹ��I2C��ģʽ
*********************************************************************************************************
*/
uint8  IRcvByte(uint8 sla, uint8 *dat)
{  /* �������� */
   I2C_sla     = sla+1;		// ��������������ַ
   I2C_buf     = dat;
   I2C_num     = 1;
   I2C_suba_en = 0;			// ���ӵ�ַ
   I2C_end     = 0;
   
   I2CONCLR = 0x2C;
   I2CONSET = 0x60;         // ����Ϊ����������������
   
   while(0==I2C_end);
   if(1==I2C_end) return(1);
     else return(0);
}


/*
*********************************************************************************************************
** �������� ��I2C_ReadNByte()
** �������� �������ӵ�ַ���������ַ��ʼ��ȡN�ֽ�����
** ��ڲ��� ��	sla			�����ӵ�ַ
**				suba_type	�ӵ�ַ�ṹ	1�����ֽڵ�ַ	2��8+X�ṹ	2��˫�ֽڵ�ַ
**				suba		�����ӵ�ַ
**				s			���ݽ��ջ�����ָ��
**				num			��ȡ�ĸ���
** ���ڲ��� ��	TRUE		�����ɹ�
**				FALSE		����ʧ��
*********************************************************************************************************
*/
uint8 I2C_ReadNByte (uint8 sla, uint32 suba_type, uint32 suba, uint8 *s, uint32 num)
{
	if (num > 0)	/* �ж�num�����ĺϷ��� */
	{	/* �������� */
		if (suba_type == 1)
		{	/* �ӵ�ַΪ���ֽ� */
			I2C_sla     	= sla + 1;							/* �������Ĵӵ�ַ��R=1 	*/
			I2C_suba    	= suba;								/* �����ӵ�ַ 			*/
			I2C_suba_num	= 1;								/* �����ӵ�ַΪ1�ֽ� 	*/
		}
		if (suba_type == 2)
		{	/* �ӵ�ַΪ2�ֽ� */
			I2C_sla     	= sla + 1;							/* �������Ĵӵ�ַ��R=1 	*/
			I2C_suba   	 	= suba;								/* �����ӵ�ַ 			*/
			I2C_suba_num	= 2;								/* �����ӵ�ַΪ2�ֽ� 	*/
		}
		if (suba_type == 3)
		{	/* �ӵ�ַ�ṹΪ8+X*/
			I2C_sla			= sla + ((suba >> 7 )& 0x0e) + 1;	/* �������Ĵӵ�ַ��R=1	*/
			I2C_suba		= suba & 0x0ff;						/* �����ӵ�ַ	 		*/
			I2C_suba_num	= 1;								/* �����ӵ�ַΪ8+x	 	*/
		}
		I2C_buf     = s;										/* ���ݽ��ջ�����ָ�� 	*/
		I2C_num     = num;										/* Ҫ��ȡ�ĸ��� 		*/
		I2C_suba_en = 1;										/* ���ӵ�ַ�� 			*/
		I2C_end     = 0;
		
		/* ���STA,SI,AA��־λ */
		I2CONCLR = 	(1 << 2)|	/* AA 		*/
					(1 << 3)|	/* SI 		*/
					(1 << 5);	/* STA 		*/
		
		/* ��λSTA,����I2C���� */
		I2CONSET = 	(1 << 5)|	/* STA 		*/
					(1 << 6);	/* I2CEN 	*/
		
		/* �ȴ�I2C������� */
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
** �������� ��I2C_WriteNByte()
** �������� �������ӵ�ַ����д��N�ֽ�����
** ��ڲ��� ��	sla			�����ӵ�ַ
**				suba_type	�ӵ�ַ�ṹ	1�����ֽڵ�ַ	3��8+X�ṹ	2��˫�ֽڵ�ַ
**			  	suba		�����ڲ������ַ
**			  	*s			��Ҫд������ݵ�ָ��
**			  	num			��Ҫд������ݵĸ���
** ���ڲ��� ��	TRUE		�����ɹ�
**			  	FALSE		����ʧ��
*********************************************************************************************************
*/
uint8 I2C_WriteNByte(uint8 sla, uint8 suba_type, uint32 suba, uint8 *s, uint32 num)
{
	if (num > 0)/* �����ȡ�ĸ���Ϊ0���򷵻ش��� */
	{	/* ���ò��� */	
		if (suba_type == 1)
		{	/* �ӵ�ַΪ���ֽ� */
			I2C_sla     	= sla;								/* �������Ĵӵ�ַ	 	*/
			I2C_suba    	= suba;								/* �����ӵ�ַ 			*/
			I2C_suba_num	= 1;								/* �����ӵ�ַΪ1�ֽ� 	*/
		}
		if (suba_type == 2)
		{	/* �ӵ�ַΪ2�ֽ� */
			I2C_sla     	= sla;								/* �������Ĵӵ�ַ 		*/
			I2C_suba   	 	= suba;								/* �����ӵ�ַ 			*/
			I2C_suba_num	= 2;								/* �����ӵ�ַΪ2�ֽ� 	*/
		}
		if (suba_type == 3)
		{	/* �ӵ�ַ�ṹΪ8+X */
			I2C_sla			= sla + ((suba >> 7 )& 0x0e);		/* �������Ĵӵ�ַ		*/
			I2C_suba		= suba & 0x0ff;						/* �����ӵ�ַ			*/
			I2C_suba_num	= 1;								/* �����ӵ�ַΪ8+X	 	*/
		}

		I2C_buf     = s;										/* ���� 				*/
		I2C_num     = num;										/* ���ݸ��� 			*/
		I2C_suba_en = 2;										/* ���ӵ�ַ��д���� 	*/
		I2C_end     = 0;
		
		/* ���STA,SI,AA��־λ */
		I2CONCLR = 	(1 << 2)|	/* AA 	*/
					(1 << 3)|	/* SI 	*/
					(1 << 5);	/* STA 	*/
		
		/* ��λSTA,����I2C���� */
		I2CONSET = 	(1 << 5)|	/* STA 	*/
					(1 << 6);	/* I2CEN*/
		
		/* �ȴ�I2C������� */
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
** �������� ��__irq IRQ_I2C()
** �������� ��Ӳ��I2C�жϷ������ 
** ��ڲ��� ����
** ���ڲ��� ����
** ˵��     ��ע�⴦���ӵ�ַΪ2�ֽڵ������ 
*********************************************************************************************************
*/
void __irq IRQ_I2C(void)
{	/* ��ȡI2C״̬�Ĵ���I2DAT */
	/* ����ȫ�ֱ��������ý��в��������������־ */
	/* ����ж��߼�,�жϷ��� */
	
	switch (I2STAT & 0xF8)
	{	/* ����״̬�������Ӧ�Ĵ��� */
		case 0x08:	/* �ѷ�����ʼ���� */				/* �����ͺ������ն��� 		*/
			/* װ��SLA+W����SLA+R */
		 	if(I2C_suba_en == 1)/* SLA+R */				/* ָ���ӵ�ַ�� 			*/
		 	{	I2DAT = I2C_sla & 0xFE; 				/* ��д���ַ 				*/
		 	}
            else	/* SLA+W */
            {  	I2DAT = I2C_sla;        				/* ����ֱ�ӷ��ʹӻ���ַ 	*/
            }
            /* ����SIλ */
            I2CONCLR =	(1 << 3)|						/* SI 						*/
            			(1 << 5);						/* STA 						*/
            break;
            
       	case 0x10:	/*�ѷ����ظ���ʼ���� */ 			/* �����ͺ������ն��� 		*/
       		/* װ��SLA+W����SLA+R */
       		I2DAT = I2C_sla;							/* �������ߺ��ط��ӵ�ַ 	*/
       		I2CONCLR = 0x28;							/* ����SI,STA */
       		break;

		case 0x18:
       	case 0x28:	/* �ѷ���I2DAT�е����ݣ��ѽ���ACK */
       		if (I2C_suba_en == 0)
       		{
	       		if (I2C_num > 0)
	       		{	I2DAT = *I2C_buf++;
	       			I2CONCLR = 0x28;					/* ����SI,STA 				*/
	       			I2C_num--;
	       		}
	       		else	/* û�����ݷ����� */
	       		{		/* ֹͣ���� */
	       		  	I2CONSET = (1 << 4);				/* STO 						*/
	       			I2CONCLR = 0x28;					/* ����SI,STA 				*/
	       		  	I2C_end = 1;						/* �����Ѿ�ֹͣ 			*/
	       		}
       		}
       		
            if(I2C_suba_en == 1)	/* ����ָ����ַ������������������ 				*/
            { 
            	if (I2C_suba_num == 2)
            	{	I2DAT = ((I2C_suba >> 8) & 0xff);
	       			I2CONCLR = 0x28;					/* ����SI,STA 				*/
	       			I2C_suba_num--;
	       			break;	
	       		} 
	       		
	       		if(I2C_suba_num == 1)
	       		{	I2DAT = (I2C_suba & 0xff);
	       			I2CONCLR = 0x28;					/* ����SI,STA 				*/
	       			I2C_suba_num--;
	       			break;	
	       		}
	       		
            	if (I2C_suba_num == 0)
            	{	I2CONSET = 0x20;
               		I2CONCLR = 0x08;
               		I2C_suba_en = 0;     				/* �ӵ�ַ������ 			*/
               		break;
               	}
            }
            
            if (I2C_suba_en == 2)/* ָ���ӵ�ַд,�ӵ�ַ��δָ��,�����ӵ�ַ 		*/
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
       		  
       case 0x40:	/* �ѷ���SLA+R,�ѽ���ACK */
       		if (I2C_num <= 1)	/* ��������һ���ֽ� */			
       		{	I2CONCLR = 1 << 2;      				/* �´η��ͷ�Ӧ���ź� 		*/
       		}
       		else
       		{ 	I2CONSET = 1 << 2;						/* �´η���Ӧ���ź� 		*/
       		}
       		I2CONCLR = 0x28;							/* ����SI,STA 				*/
       		break;

       	case 0x20:	/* �ѷ���SLA+W,�ѽ��շ�Ӧ��              */
       	case 0x30:	/* �ѷ���I2DAT�е����ݣ��ѽ��շ�Ӧ��     */
       	case 0x38:	/* ��SLA+R/W�������ֽ��ж�ʧ�ٲ�         */
   		case 0x48:	/* �ѷ���SLA+R,�ѽ��շ�Ӧ��              */
         	I2CONCLR = 0x28;
            I2C_end = 0xFF; 
       		break;   				
	
		case 0x50:	/* �ѽ��������ֽڣ��ѷ���ACK */
			*I2C_buf++ = I2DAT;
			I2C_num--;
			if (I2C_num == 1)/* �������һ���ֽ� */
			{  	I2CONCLR = 0x2C;						/* STA,SI,AA = 0 			*/
			}
			else
			{  	I2CONSET = 0x04;						/* AA=1 					*/
			  	I2CONCLR = 0x28;
			}
			break;
		
		case 0x58:	/* �ѽ��������ֽڣ��ѷ��ط�Ӧ�� */
			*I2C_buf++ = I2DAT;     					/* ��ȡ���һ�ֽ����� 		*/
            I2CONSET = 0x10;        					/* �������� 				*/
            I2CONCLR = 0x28;
            I2C_end = 1; 
            break;
            
      	default:
      		break;
	}
   VICVectAddr = 0x00;              					/* �жϴ������ 			*/
}


/*******************************************SC16IS752����***********************************************************************************/

/****************************************************************************
* SPI_Wr_752(uint8 reg, uint8 data, uint8 channel)
* Function: Write SC16IS762
* Input�� uint8 reg, uint8 data, uint8 slaver
* output: none
****************************************************************************/
void SPI_wr_752(uint8 reg, uint8 data, uint8 channel)
{
	char str[2];
  reg <<= 3; // �Ĵ�����ַ

  if ((channel==0x01)) 
   reg |= 0x02; // ͨ����ַ
  
	str[0]=data;
  I2C_WriteNByte(Adr752, 1, reg, str, 1);

}

/****************************************************************************
* uint8 SPI_rd_752(uint8 reg, uint8 channel)
* Function: Read SC16IS762
* Input�� uint8 reg, uint8 channel
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
/* �������ƣ�Set_GPIO_Dir();                               */
/* ����������SC16IS752_IODir  IO����Ĵ���  [7:0]  0���� 1��� */
/*			 SC16IS752_IOControl bit0 IO���� bit1 gpio[7:4]/modem bit2 gpio[3:0]/modem 0 io 1modem */
/* ����˵����logic   �ܽ����뻹����� */
/*              channel   ����ͨ��                          */
/* �������أ��� */
/******************************************************************************/
void Set_GPIO_Dir(uint8 channel,uint8 logic)                                                                  
{ // Set Direction on UART GPIO Port pins GPIO0 to GPIO7 
     // 0=input   1=Output  
  SPI_wr_752 (SC16IS752_LCR,SPI_rd_752(SC16IS752_LCR,channel)&0x7F, channel);
	SPI_wr_752 (SC16IS752_IOControl, 0x03,channel); // Set the IOControl Register to GPIO Control 
	SPI_wr_752 (SC16IS752_IODir,logic,channel); // output the control bits to the IO Direction Register 
} 
/******************************************************************************/
/* �������ƣ�Read_GPIO();                               */
/* �������������ܽ�״ֵ̬      �����عܽ�״̬                       */
/* ����˵����channel   ����ͨ��	*/
/* �������أ��������� */
  
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
/* �������ƣ�Write_GPIO();                               */
/* ����������д�ܽ�״ֵ̬     дSC16IS752_IOState 0���0 1���1                       */
/* ����˵����reg   �Ĵ�����ַ
              channel   ����ͨ��                          */
/* �������أ��������� */
  
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
/* �������ƣ�SC16IS752_Init_ChA1_232();                               */
/* ������������ʼ��SC16IS752�˿�a                           */
/* ����˵������                          */
/* �������أ��� */
  
/******************************************************************************/
void SC16IS752_Init_ChA1_232 (void) 

{

SPI_wr_752 (SC16IS752_LCR, 0x80, 0);
SPI_wr_752 (SC16IS752_DLL, 1, 0);  //115200
SPI_wr_752 (SC16IS752_DLH, 0x00, 0);  
SPI_wr_752 (SC16IS752_LCR, 0xBF, 0); //1011 1111 
SPI_wr_752 (SC16IS752_EFR, 0x10, 0);
SPI_wr_752 (SC16IS752_LCR, 0x03, 0);
SPI_wr_752 (SC16IS752_FCR, 0x07, 0); //���ա����ʹ�����Ϊ8�ֽ�
SPI_wr_752 (SC16IS752_SPR, 'A', 0);

SPI_wr_752 (SC16IS752_IER, 0x01, 0); 

} 
/******************************************************************************/
/* �������ƣ�SC16IS752_Init_ChB1_232();                               */
/* ������������ʼ��SC16IS752�˿�b                           */
/* ����˵������                          */
/* �������أ��� */
  
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

	