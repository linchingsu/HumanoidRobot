/****************************************Copyright (c)**************************************************
**                               ������������Ƭ����չ���޹�˾
**                                     ��    ��    ��
**                                        ��Ʒһ�� 
**
**                                 http://www.zlgmcu.com
**
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**��   ��   ��: LPC214x.h
**��   ��   ��: ������
**����޸�����: 2005-09-19
**��        ��: ����lpc23x/LPC214x������Ĵ������̼�����
**
**--------------��ʷ�汾��Ϣ----------------------------------------------------------------------------
** ������: ������
** ��  ��: v1.0
** �ա���: 2005-09-19
** �衡��: ԭʼ�汾������������д���������û��ֲ�����޸ģ���ϣ���������e-mail: lpc2130@zlgmcu.com
**
**--------------��ǰ�汾�޶�------------------------------------------------------------------------------
** �޸���: 
** �ա���:
** �衡��:
**
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

/* External Interrupts */
/* �ⲿ�жϿ��ƼĴ��� */
#define EXTINT          (*((volatile unsigned char *) 0xE01FC140))
#define INTWAKE         (*((volatile unsigned long *) 0xE01FC144))
#define EXTMODE         (*((volatile unsigned char *) 0xE01FC148))
#define EXTPOLAR        (*((volatile unsigned char *) 0xE01FC14C))

/* Memory mapping control */
/* �ڴ�remap���ƼĴ��� */
#define MEMMAP          (*((volatile unsigned long *) 0xE01FC040))

/* Phase Locked Loop (PLL) */
/* PLL���ƼĴ��� */
#define PLLCON          (*((volatile unsigned char *) 0xE01FC080))
#define PLLCFG          (*((volatile unsigned char *) 0xE01FC084))
#define PLLSTAT         (*((volatile unsigned short*) 0xE01FC088))
#define PLLFEED         (*((volatile unsigned char *) 0xE01FC08C))

/* PLL0 (PLL) */
#define PLL0CON          (*((volatile unsigned char *) 0xE01FC080))
#define PLL0CFG          (*((volatile unsigned char *) 0xE01FC084))
#define PLL0STAT         (*((volatile unsigned short*) 0xE01FC088))
#define PLL0FEED         (*((volatile unsigned char *) 0xE01FC08C))

/* USB PLL1 Register */
#define PLL1CON        	(*((volatile unsigned char *) 0xE01FC0A0))
#define PLL1CFG         (*((volatile unsigned char *) 0xE01FC0A4))
#define PLL1STAT        (*((volatile unsigned short*) 0xE01FC0A8))
#define PLL1FEED        (*((volatile unsigned char *) 0xE01FC0AC))

/* Power Control */
/* ���ʿ��ƼĴ��� */
#define PCON            (*((volatile unsigned char *) 0xE01FC0C0))
#define PCONP           (*((volatile unsigned long *) 0xE01FC0C4))

/* VPB Divider */
/* VLSI�������ߣ�VPB����Ƶ�Ĵ��� */
#define VPBDIV          (*((volatile unsigned char *) 0xE01FC100))

/* Reset Source Identification Register */ 
/* ��λԴʶ��Ĵ��� */
#define RSIR            (*((volatile unsigned char *) 0xE01FC180))

/* Code Security Protection Register */
/* ���밲ȫ�Ĵ��� */
#define CSPR            (*((volatile unsigned char *) 0xE01FC184))

/* ����ϵͳ���ƼĴ��� */
#define SCS 		    (*((volatile unsigned long *) 0xE01FC1A0))

/* Memory Accelerator Module (MAM) */
/* �洢������ģ�� */
#define MAMCR           (*((volatile unsigned char *) 0xE01FC000))
#define MAMTIM          (*((volatile unsigned char *) 0xE01FC004))

/* Vectored Interrupt Controller (VIC) */
/* �����жϿ�����(VIC)������Ĵ��� */
#define VICIRQStatus    (*((volatile unsigned long *) 0xFFFFF000))
#define VICFIQStatus    (*((volatile unsigned long *) 0xFFFFF004))
#define VICRawIntr      (*((volatile unsigned long *) 0xFFFFF008))
#define VICIntSelect    (*((volatile unsigned long *) 0xFFFFF00C))
#define VICIntEnable    (*((volatile unsigned long *) 0xFFFFF010))
#define VICIntEnClr     (*((volatile unsigned long *) 0xFFFFF014))
#define VICSoftInt      (*((volatile unsigned long *) 0xFFFFF018))
#define VICSoftIntClear (*((volatile unsigned long *) 0xFFFFF01C))
#define VICProtection   (*((volatile unsigned long *) 0xFFFFF020))
#define VICVectAddr     (*((volatile unsigned long *) 0xFFFFF030))
#define VICDefVectAddr  (*((volatile unsigned long *) 0xFFFFF034))
#define VICVectAddr0    (*((volatile unsigned long *) 0xFFFFF100))
#define VICVectAddr1    (*((volatile unsigned long *) 0xFFFFF104))
#define VICVectAddr2    (*((volatile unsigned long *) 0xFFFFF108))
#define VICVectAddr3    (*((volatile unsigned long *) 0xFFFFF10C))
#define VICVectAddr4    (*((volatile unsigned long *) 0xFFFFF110))
#define VICVectAddr5    (*((volatile unsigned long *) 0xFFFFF114))
#define VICVectAddr6    (*((volatile unsigned long *) 0xFFFFF118))
#define VICVectAddr7    (*((volatile unsigned long *) 0xFFFFF11C))
#define VICVectAddr8    (*((volatile unsigned long *) 0xFFFFF120))
#define VICVectAddr9    (*((volatile unsigned long *) 0xFFFFF124))
#define VICVectAddr10   (*((volatile unsigned long *) 0xFFFFF128))
#define VICVectAddr11   (*((volatile unsigned long *) 0xFFFFF12C))
#define VICVectAddr12   (*((volatile unsigned long *) 0xFFFFF130))
#define VICVectAddr13   (*((volatile unsigned long *) 0xFFFFF134))
#define VICVectAddr14   (*((volatile unsigned long *) 0xFFFFF138))
#define VICVectAddr15   (*((volatile unsigned long *) 0xFFFFF13C))
#define VICVectCntl0    (*((volatile unsigned long *) 0xFFFFF200))
#define VICVectCntl1    (*((volatile unsigned long *) 0xFFFFF204))
#define VICVectCntl2    (*((volatile unsigned long *) 0xFFFFF208))
#define VICVectCntl3    (*((volatile unsigned long *) 0xFFFFF20C))
#define VICVectCntl4    (*((volatile unsigned long *) 0xFFFFF210))
#define VICVectCntl5    (*((volatile unsigned long *) 0xFFFFF214))
#define VICVectCntl6    (*((volatile unsigned long *) 0xFFFFF218))
#define VICVectCntl7    (*((volatile unsigned long *) 0xFFFFF21C))
#define VICVectCntl8    (*((volatile unsigned long *) 0xFFFFF220))
#define VICVectCntl9    (*((volatile unsigned long *) 0xFFFFF224))
#define VICVectCntl10   (*((volatile unsigned long *) 0xFFFFF228))
#define VICVectCntl11   (*((volatile unsigned long *) 0xFFFFF22C))
#define VICVectCntl12   (*((volatile unsigned long *) 0xFFFFF230))
#define VICVectCntl13   (*((volatile unsigned long *) 0xFFFFF234))
#define VICVectCntl14   (*((volatile unsigned long *) 0xFFFFF238))
#define VICVectCntl15   (*((volatile unsigned long *) 0xFFFFF23C))

/* Pin Connect Block */
/* �ܽ�����ģ����ƼĴ��� */
#define PINSEL0         (*((volatile unsigned long *) 0xE002C000))
#define PINSEL1         (*((volatile unsigned long *) 0xE002C004))
#define PINSEL2         (*((volatile unsigned long *) 0xE002C014))

/* General Purpose Input/Output (GPIO) */
/* ͨ�ò���IO�ڵ�����Ĵ��� */
#define IO0PIN          (*((volatile unsigned long *) 0xE0028000))
#define IO0SET          (*((volatile unsigned long *) 0xE0028004))
#define IO0DIR          (*((volatile unsigned long *) 0xE0028008))
#define IO0CLR          (*((volatile unsigned long *) 0xE002800C))

#define IO1PIN          (*((volatile unsigned long *) 0xE0028010))
#define IO1SET          (*((volatile unsigned long *) 0xE0028014))
#define IO1DIR          (*((volatile unsigned long *) 0xE0028018))
#define IO1CLR          (*((volatile unsigned long *) 0xE002801C))

/* FAST GPIO Registers */
/* ��Ѱַ�ļĴ��� */
#define FIO0DIR			(*((volatile unsigned long *) 0x3FFFC000))
#define FIO0MASK		(*((volatile unsigned long *) 0x3FFFC010))
#define FIO0PIN			(*((volatile unsigned long *) 0x3FFFC014))
#define FIO0SET			(*((volatile unsigned long *) 0x3FFFC018))
#define FIO0CLR			(*((volatile unsigned long *) 0x3FFFC01C))

#define FIO1DIR			(*((volatile unsigned long *) 0x3FFFC020))
#define FIO1MASK		(*((volatile unsigned long *) 0x3FFFC030))
#define FIO1PIN			(*((volatile unsigned long *) 0x3FFFC034))
#define FIO1SET			(*((volatile unsigned long *) 0x3FFFC038))
#define FIO1CLR			(*((volatile unsigned long *) 0x3FFFC03C))

/* ����Ѱַ�Ĵ��� */
#define FIO0DIRL		(*((volatile unsigned short*) 0x3FFFC000))
#define FIO0DIRU		(*((volatile unsigned short*) 0x3FFFC002))
#define FIO0MASKL		(*((volatile unsigned short*) 0x3FFFC010))
#define FIO0MASKU		(*((volatile unsigned short*) 0x3FFFC012))
#define FIO0PINL		(*((volatile unsigned short*) 0x3FFFC014))
#define FIO0PINU		(*((volatile unsigned short*) 0x3FFFC016))
#define FIO0SETL		(*((volatile unsigned short*) 0x3FFFC018))
#define FIO0SETU		(*((volatile unsigned short*) 0x3FFFC01A))
#define FIO0CLRL		(*((volatile unsigned short*) 0x3FFFC01C))
#define FIO0CLRU		(*((volatile unsigned short*) 0x3FFFC01E))

#define FIO1DIRL		(*((volatile unsigned short*) 0x3FFFC020))
#define FIO1DIRU		(*((volatile unsigned short*) 0x3FFFC022))
#define FIO1MASKL		(*((volatile unsigned short*) 0x3FFFC030))
#define FIO1MASKU		(*((volatile unsigned short*) 0x3FFFC032))
#define FIO1PINL		(*((volatile unsigned short*) 0x3FFFC034))
#define FIO1PINU		(*((volatile unsigned short*) 0x3FFFC036))
#define FIO1SETL		(*((volatile unsigned short*) 0x3FFFC038))
#define FIO1SETU		(*((volatile unsigned short*) 0x3FFFC03A))
#define FIO1CLRL		(*((volatile unsigned short*) 0x3FFFC03C))
#define FIO1CLRU		(*((volatile unsigned short*) 0x3FFFC03E))

/* �ֽ�Ѱַ�ļĴ��� */
#define FIO0DIR0		(*((volatile unsigned char *) 0x3FFFC000))
#define FIO0DIR1		(*((volatile unsigned char *) 0x3FFFC001))
#define FIO0DIR2		(*((volatile unsigned char *) 0x3FFFC002))
#define FIO0DIR3		(*((volatile unsigned char *) 0x3FFFC003))
#define FIO0MASK0		(*((volatile unsigned char *) 0x3FFFC010))
#define FIO0MASK1		(*((volatile unsigned char *) 0x3FFFC011))
#define FIO0MASK2		(*((volatile unsigned char *) 0x3FFFC012))
#define FIO0MASK3		(*((volatile unsigned char *) 0x3FFFC013))
#define FIO0PIN0		(*((volatile unsigned char *) 0x3FFFC014))
#define FIO0PIN1		(*((volatile unsigned char *) 0x3FFFC015))
#define FIO0PIN2		(*((volatile unsigned char *) 0x3FFFC016))
#define FIO0PIN3		(*((volatile unsigned char *) 0x3FFFC017))
#define FIO0SET0		(*((volatile unsigned char *) 0x3FFFC018))
#define FIO0SET1		(*((volatile unsigned char *) 0x3FFFC019))
#define FIO0SET2		(*((volatile unsigned char *) 0x3FFFC01A))
#define FIO0SET3		(*((volatile unsigned char *) 0x3FFFC01B))
#define FIO0CLR0		(*((volatile unsigned char *) 0x3FFFC01C))
#define FIO0CLR1		(*((volatile unsigned char *) 0x3FFFC01D))
#define FIO0CLR2		(*((volatile unsigned char *) 0x3FFFC01E))
#define FIO0CLR3		(*((volatile unsigned char *) 0x3FFFC01F))

#define FIO1DIR0		(*((volatile unsigned char *) 0x3FFFC020))
#define FIO1DIR1		(*((volatile unsigned char *) 0x3FFFC021))
#define FIO1DIR2		(*((volatile unsigned char *) 0x3FFFC022))
#define FIO1DIR3		(*((volatile unsigned char *) 0x3FFFC023))
#define FIO1MASK0		(*((volatile unsigned char *) 0x3FFFC030))
#define FIO1MASK1		(*((volatile unsigned char *) 0x3FFFC031))
#define FIO1MASK2		(*((volatile unsigned char *) 0x3FFFC032))
#define FIO1MASK3		(*((volatile unsigned char *) 0x3FFFC033))
#define FIO1PIN0		(*((volatile unsigned char *) 0x3FFFC034))
#define FIO1PIN1		(*((volatile unsigned char *) 0x3FFFC035))
#define FIO1PIN2		(*((volatile unsigned char *) 0x3FFFC036))
#define FIO1PIN3		(*((volatile unsigned char *) 0x3FFFC037))
#define FIO1SET0		(*((volatile unsigned char *) 0x3FFFC038))
#define FIO1SET1		(*((volatile unsigned char *) 0x3FFFC039))
#define FIO1SET2		(*((volatile unsigned char *) 0x3FFFC03A))
#define FIO1SET3		(*((volatile unsigned char *) 0x3FFFC03B))
#define FIO1CLR0		(*((volatile unsigned char *) 0x3FFFC03C))
#define FIO1CLR1		(*((volatile unsigned char *) 0x3FFFC03D))
#define FIO1CLR2		(*((volatile unsigned char *) 0x3FFFC03E))
#define FIO1CLR3		(*((volatile unsigned char *) 0x3FFFC03F))

/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
/* ͨ���첽���п�0(UART0)������Ĵ��� */
#define U0RBR           (*((volatile unsigned char *) 0xE000C000))
#define U0THR           (*((volatile unsigned char *) 0xE000C000))
#define U0IER           (*((volatile unsigned char *) 0xE000C004))
#define U0IIR           (*((volatile unsigned char *) 0xE000C008))
#define U0FCR           (*((volatile unsigned char *) 0xE000C008))
#define U0LCR           (*((volatile unsigned char *) 0xE000C00C))
#define U0LSR           (*((volatile unsigned char *) 0xE000C014))
#define U0SCR           (*((volatile unsigned char *) 0xE000C01C))
#define U0ACR           (*((volatile unsigned long *) 0xE000C020))
#define U0FDR           (*((volatile unsigned long *) 0xE000C028))
#define U0TER           (*((volatile unsigned char *) 0xE000C030))
#define U0DLL           (*((volatile unsigned char *) 0xE000C000))
#define U0DLM           (*((volatile unsigned char *) 0xE000C004))

/* Universal Asynchronous Receiver Transmitter 1 (UART1) */
/* ͨ���첽���п�1(UART1)������Ĵ��� */
#define U1RBR           (*((volatile unsigned char *) 0xE0010000))
#define U1THR           (*((volatile unsigned char *) 0xE0010000))
#define U1IER           (*((volatile unsigned char *) 0xE0010004))
#define U1IIR           (*((volatile unsigned char *) 0xE0010008))
#define U1FCR           (*((volatile unsigned char *) 0xE0010008))
#define U1LCR           (*((volatile unsigned char *) 0xE001000C))
#define U1MCR           (*((volatile unsigned char *) 0xE0010010))	/* LPC2144/6/8 */
#define U1LSR           (*((volatile unsigned char *) 0xE0010014))
#define U1MSR           (*((volatile unsigned char *) 0xE0010018))	/* LPC2144/6/8 */
#define U1SCR           (*((volatile unsigned char *) 0xE001001C))
#define U1ACR           (*((volatile unsigned long *) 0xE0010020))
#define U1FDR           (*((volatile unsigned long *) 0xE0010028))
#define U1TER           (*((volatile unsigned char *) 0xE0010030))
#define U1DLL           (*((volatile unsigned char *) 0xE0010000))
#define U1DLM           (*((volatile unsigned char *) 0xE0010004))


/* I2C (8/16 bit data bus) */
/* оƬ�����ߣ�I2C��������Ĵ��� */
#define I2CONSET        (*((volatile unsigned long *) 0xE001C000))
#define I2STAT          (*((volatile unsigned long *) 0xE001C004))
#define I2DAT           (*((volatile unsigned long *) 0xE001C008))
#define I2ADR           (*((volatile unsigned long *) 0xE001C00C))
#define I2SCLH          (*((volatile unsigned long *) 0xE001C010))
#define I2SCLL          (*((volatile unsigned long *) 0xE001C014))
#define I2CONCLR        (*((volatile unsigned long *) 0xE001C018))

/* I2C0 (8/16 bit data bus) */
/* оƬ�����ߣ�I2C0��������Ĵ��� */
#define I2C0CONSET      (*((volatile unsigned long *) 0xE001C000))
#define I2C0STAT        (*((volatile unsigned long *) 0xE001C004))
#define I2C0DAT         (*((volatile unsigned long *) 0xE001C008))
#define I2C0ADR         (*((volatile unsigned long *) 0xE001C00C))
#define I2C0SCLH        (*((volatile unsigned long *) 0xE001C010))
#define I2C0SCLL        (*((volatile unsigned long *) 0xE001C014))
#define I2C0CONCLR      (*((volatile unsigned long *) 0xE001C018))

/* I2C1 (8/16 bit data bus) */
/* оƬ�����ߣ�I2C1��������Ĵ��� */
#define I2C1CONSET      (*((volatile unsigned long *) 0xE005C000))
#define I2C1STAT        (*((volatile unsigned long *) 0xE005C004))
#define I2C1DAT         (*((volatile unsigned long *) 0xE005C008))
#define I2C1ADR         (*((volatile unsigned long *) 0xE005C00C))
#define I2C1SCLH        (*((volatile unsigned long *) 0xE005C010))
#define I2C1SCLL        (*((volatile unsigned long *) 0xE005C014))
#define I2C1CONCLR      (*((volatile unsigned long *) 0xE005C018))

/* SPI (Serial Peripheral Interface) */
/* SPI���߽ӿڵ�����Ĵ��� */
#define S0SPCR        (*((volatile unsigned short*) 0xE0020000))
#define S0SPSR        (*((volatile unsigned char *) 0xE0020004))
#define S0SPDR        (*((volatile unsigned short*) 0xE0020008))
#define S0SPCCR       (*((volatile unsigned char *) 0xE002000C))
#define S0SPINT       (*((volatile unsigned char *) 0xE002001C))

/* SSP Registers */ 
#define SSPCR0          (*((volatile unsigned short*) 0xE0068000))
#define SSPCR1          (*((volatile unsigned char *) 0xE0068004))
#define SSPDR           (*((volatile unsigned short*) 0xE0068008))
#define SSPSR           (*((volatile unsigned char *) 0xE006800C))
#define SSPCPSR         (*((volatile unsigned char *) 0xE0068010))
#define SSPIMSC         (*((volatile unsigned char *) 0xE0068014))
#define SSPRIS          (*((volatile unsigned char *) 0xE0068018))
#define SSPMIS          (*((volatile unsigned char *) 0xE006801C))
#define SSPICR          (*((volatile unsigned char *) 0xE0068020))

/* Timer 0 */
/* ��ʱ��0������Ĵ��� */
#define T0IR            (*((volatile unsigned long *) 0xE0004000))
#define T0TCR           (*((volatile unsigned long *) 0xE0004004))
#define T0TC            (*((volatile unsigned long *) 0xE0004008))
#define T0PR            (*((volatile unsigned long *) 0xE000400C))
#define T0PC            (*((volatile unsigned long *) 0xE0004010))
#define T0MCR           (*((volatile unsigned long *) 0xE0004014))
#define T0MR0           (*((volatile unsigned long *) 0xE0004018))
#define T0MR1           (*((volatile unsigned long *) 0xE000401C))
#define T0MR2           (*((volatile unsigned long *) 0xE0004020))
#define T0MR3           (*((volatile unsigned long *) 0xE0004024))
#define T0CCR           (*((volatile unsigned long *) 0xE0004028))
#define T0CR0           (*((volatile unsigned long *) 0xE000402C))
#define T0CR1           (*((volatile unsigned long *) 0xE0004030))
#define T0CR2           (*((volatile unsigned long *) 0xE0004034))
#define T0CR3           (*((volatile unsigned long *) 0xE0004038))
#define T0EMR           (*((volatile unsigned long *) 0xE000403C))
#define T0CTCR          (*((volatile unsigned long *) 0xE0004070))

/* Timer 1 */
/* ��ʱ��1������Ĵ��� */
#define T1IR            (*((volatile unsigned long *) 0xE0008000))
#define T1TCR           (*((volatile unsigned long *) 0xE0008004))
#define T1TC            (*((volatile unsigned long *) 0xE0008008))
#define T1PR            (*((volatile unsigned long *) 0xE000800C))
#define T1PC            (*((volatile unsigned long *) 0xE0008010))
#define T1MCR           (*((volatile unsigned long *) 0xE0008014))
#define T1MR0           (*((volatile unsigned long *) 0xE0008018))
#define T1MR1           (*((volatile unsigned long *) 0xE000801C))
#define T1MR2           (*((volatile unsigned long *) 0xE0008020))
#define T1MR3           (*((volatile unsigned long *) 0xE0008024))
#define T1CCR           (*((volatile unsigned long *) 0xE0008028))
#define T1CR0           (*((volatile unsigned long *) 0xE000802C))
#define T1CR1           (*((volatile unsigned long *) 0xE0008030))
#define T1CR2           (*((volatile unsigned long *) 0xE0008034))
#define T1CR3           (*((volatile unsigned long *) 0xE0008038))
#define T1EMR           (*((volatile unsigned long *) 0xE000803C))
#define T1CTCR          (*((volatile unsigned long *) 0xE0008070))

/* Pulse Width Modulator (PWM) */
/* ���������������Ĵ��� */
#define PWMIR           (*((volatile unsigned long *) 0xE0014000))
#define PWMTCR          (*((volatile unsigned long *) 0xE0014004))
#define PWMTC           (*((volatile unsigned long *) 0xE0014008))
#define PWMPR           (*((volatile unsigned long *) 0xE001400C))
#define PWMPC           (*((volatile unsigned long *) 0xE0014010))
#define PWMMCR          (*((volatile unsigned long *) 0xE0014014))
#define PWMMR0          (*((volatile unsigned long *) 0xE0014018))
#define PWMMR1          (*((volatile unsigned long *) 0xE001401C))
#define PWMMR2          (*((volatile unsigned long *) 0xE0014020))
#define PWMMR3          (*((volatile unsigned long *) 0xE0014024))
#define PWMMR4          (*((volatile unsigned long *) 0xE0014040))
#define PWMMR5          (*((volatile unsigned long *) 0xE0014044))
#define PWMMR6          (*((volatile unsigned long *) 0xE0014048))
#define PWMPCR          (*((volatile unsigned long *) 0xE001404C))
#define PWMLER          (*((volatile unsigned long *) 0xE0014050))

/* A/D CONVERTER */
/* A/Dת���� */
#define ADCR            (*((volatile unsigned long *) 0xE0034000))
#define ADDR            (*((volatile unsigned long *) 0xE0034004))

#define ADGSR           (*((volatile unsigned long *) 0xE0034008))

#define AD0CR           (*((volatile unsigned long *) 0xE0034000))
#define AD0GDR          (*((volatile unsigned long *) 0xE0034004))
#define AD0STAT         (*((volatile unsigned long *) 0xE0034030))
#define AD0INTEN        (*((volatile unsigned long *) 0xE003400C))
#define AD0DR0          (*((volatile unsigned long *) 0xE0034010))
#define AD0DR1          (*((volatile unsigned long *) 0xE0034014))
#define AD0DR2          (*((volatile unsigned long *) 0xE0034018))
#define AD0DR3          (*((volatile unsigned long *) 0xE003401C))
#define AD0DR4          (*((volatile unsigned long *) 0xE0034020))
#define AD0DR5          (*((volatile unsigned long *) 0xE0034024))
#define AD0DR6          (*((volatile unsigned long *) 0xE0034028))
#define AD0DR7          (*((volatile unsigned long *) 0xE003402C))

#define AD1CR           (*((volatile unsigned long *) 0xE0060000))
#define AD1GDR          (*((volatile unsigned long *) 0xE0060004))
#define AD1STAT         (*((volatile unsigned long *) 0xE0060030))
#define AD1INTEN        (*((volatile unsigned long *) 0xE006000C))
#define AD1DR0          (*((volatile unsigned long *) 0xE0060010))
#define AD1DR1          (*((volatile unsigned long *) 0xE0060014))
#define AD1DR2          (*((volatile unsigned long *) 0xE0060018))
#define AD1DR3          (*((volatile unsigned long *) 0xE006001C))
#define AD1DR4          (*((volatile unsigned long *) 0xE0060020))
#define AD1DR5          (*((volatile unsigned long *) 0xE0060024))
#define AD1DR6          (*((volatile unsigned long *) 0xE0060028))
#define AD1DR7          (*((volatile unsigned long *) 0xE006002C))    

/* D/A CONVERTER */
/* D/Aת���� */
#define DACR            (*((volatile unsigned long *) 0xE006C000))

/* Real Time Clock */
/* ʵʱʱ�ӵ�����Ĵ��� */
#define ILR             (*((volatile unsigned char *) 0xE0024000))
#define CTC             (*((volatile unsigned short*) 0xE0024004))
#define CCR             (*((volatile unsigned char *) 0xE0024008))
#define CIIR            (*((volatile unsigned char *) 0xE002400C))
#define AMR             (*((volatile unsigned char *) 0xE0024010))
#define CTIME0          (*((volatile unsigned long *) 0xE0024014))
#define CTIME1          (*((volatile unsigned long *) 0xE0024018))
#define CTIME2          (*((volatile unsigned long *) 0xE002401C))
#define SEC             (*((volatile unsigned char *) 0xE0024020))
#define MIN             (*((volatile unsigned char *) 0xE0024024))
#define HOUR            (*((volatile unsigned char *) 0xE0024028))
#define DOM             (*((volatile unsigned char *) 0xE002402C))
#define DOW             (*((volatile unsigned char *) 0xE0024030))
#define DOY             (*((volatile unsigned short*) 0xE0024034))
#define MONTH           (*((volatile unsigned char *) 0xE0024038))
#define YEAR            (*((volatile unsigned short*) 0xE002403C))
#define ALSEC           (*((volatile unsigned char *) 0xE0024060))
#define ALMIN           (*((volatile unsigned char *) 0xE0024064))
#define ALHOUR          (*((volatile unsigned char *) 0xE0024068))
#define ALDOM           (*((volatile unsigned char *) 0xE002406C))
#define ALDOW           (*((volatile unsigned char *) 0xE0024070))
#define ALDOY           (*((volatile unsigned short*) 0xE0024074))
#define ALMON           (*((volatile unsigned char *) 0xE0024078))
#define ALYEAR          (*((volatile unsigned short*) 0xE002407C))
#define PREINT          (*((volatile unsigned short*) 0xE0024080))
#define PREFRAC         (*((volatile unsigned short*) 0xE0024084))

/* Watchdog */
/* ���Ź�������Ĵ��� */
#define WDMOD           (*((volatile unsigned char *) 0xE0000000))
#define WDTC            (*((volatile unsigned long *) 0xE0000004))
#define WDFEED          (*((volatile unsigned char *) 0xE0000008))
#define WDTV            (*((volatile unsigned long *) 0xE000000C))

/* USB Device Interrupt Register */
#define USBIntSt		(*((volatile unsigned long *) 0xE01FC1C0))
#define USBDevIntSt 	(*((volatile unsigned long *) 0xE0090000))
#define USBDevIntEn	    (*((volatile unsigned long *) 0xE0090004)) 
#define USBDevIntClr 	(*((volatile unsigned long *) 0xE0090008)) 
#define USBDevIntSet    (*((volatile unsigned long *) 0xE009000C)) 
#define USBDevIntPri 	(*((volatile unsigned long *) 0xE009002C)) 

/* USB Endpoint Interrupt Registers	*/
#define USBEpIntSt 		(*((volatile unsigned long *) 0xE0090030)) 
#define USBEPIntEn		(*((volatile unsigned long *) 0xE0090034)) 
#define USBEpIntClr		(*((volatile unsigned long *) 0xE0090038)) 
#define USBEpIntSet		(*((volatile unsigned long *) 0xE009003C)) 
#define USBEpIntPri		(*((volatile unsigned char *) 0xE0090040)) 

/* USB Realized Register */
#define USBReEp			(*((volatile unsigned long *) 0xE0090044))
#define USBEpInd		(*((volatile unsigned long *) 0xE0090048)) 
#define USBMaxPSize		(*((volatile unsigned long *) 0xE009004C))

/* USB Transfer Register */
#define USBRxData	 	(*((volatile unsigned long *) 0xE0090018)) 
#define USBTxData	 	(*((volatile unsigned long *) 0xE009001C)) 
#define USBRxPLen	 	(*((volatile unsigned long *) 0xE0090020)) 
#define USBTxPLen	 	(*((volatile unsigned long *) 0xE0090024)) 
#define USBCtrl		 	(*((volatile unsigned long *) 0xE0090028)) 

/* Command Register */
#define USBCmdCode	 	(*((volatile unsigned long *) 0xE0090010)) 
#define USBCmdData 		(*((volatile unsigned long *) 0xE0090014))

/* DMA Register, Only for LPC2146/48 */
#define USBDMARSt		(*((volatile unsigned long *) 0xE0090050))
#define USBDMARClr		(*((volatile unsigned long *) 0xE0090054))
#define USBDMARSet		(*((volatile unsigned long *) 0xE0090058))

#define USBUCDAH		(*((volatile unsigned long *) 0xE0090080))

#define USBEpDMASt		(*((volatile unsigned long *) 0xE0090084))
#define USBEpDMAEn		(*((volatile unsigned long *) 0xE0090088))
#define USBEpDMADis		(*((volatile unsigned long *) 0xE009008C))

#define USBDMAIntSt		(*((volatile unsigned long *) 0xE0090090))
#define USBDMAIntEn		(*((volatile unsigned long *) 0xE0090094))

#define USBEoTIntSt		(*((volatile unsigned long *) 0xE00900A0))
#define USBEoTIntClr	(*((volatile unsigned long *) 0xE00900A4))
#define USBEoTIntSet	(*((volatile unsigned long *) 0xE00900A8))

#define USBNDDRIntSt	(*((volatile unsigned long *) 0xE00900AC))
#define USBNDDRIntClr	(*((volatile unsigned long *) 0xE00900B0))
#define USBNDDRIntSet	(*((volatile unsigned long *) 0xE00900B4))

#define USBSysErrIntSt	(*((volatile unsigned long *) 0xE00900B8))
#define USBSysErrIntClr	(*((volatile unsigned long *) 0xE00900BC))
#define SUBsysErrIntSet	(*((volatile unsigned long *) 0xE00900C0))

/* Define firmware Functions */
/* ����̼����� */
#define rm_init_entry()             ((void (*)())(0x7fffff91))()
#define rm_undef_handler()          ((void (*)())(0x7fffffa0))()
#define rm_prefetchabort_handler()  ((void (*)())(0x7fffffb0))()
#define rm_dataabort_handler()      ((void (*)())(0x7fffffc0))()
#define rm_irqhandler()             ((void (*)())(0x7fffffd0))()
#define rm_irqhandler2()            ((void (*)())(0x7fffffe0))()
#define iap_entry(a, b)             ((void (*)())(0x7ffffff1))(a, b)

/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/
