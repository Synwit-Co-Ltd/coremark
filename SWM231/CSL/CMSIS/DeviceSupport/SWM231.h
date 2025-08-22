#ifndef __SWM231_H__
#define __SWM231_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers **********************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                        */
  HardFault_IRQn	          = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt				 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                  */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                  */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt              */
  
/******  Cortex-M0 specific Interrupt Numbers ************************************************/
  UART0_IRQn                = 0,
  UART1_IRQn                = 1,
  TIMR0_IRQn                = 2,
  PWM0_IRQn                 = 3,
  PWM1_IRQn                 = 4,
  PWMBRK_IRQn               = 5,
  DMA_IRQn					= 6,
  SPI0_IRQn  				= 7,
  WDT_IRQn                  = 8,
  PVD_IRQn    				= 9,
  HALL_IRQn                 = 10,
  ADC_IRQn                  = 11,
  ACMP_IRQn                 = 12,
  BTIMR0_IRQn               = 13,
  BTIMR1_IRQn               = 14,
  BTIMR2_IRQn               = 15,
  DIV_IRQn   				= 16,
  XTALSTOP_IRQn             = 17,
  FOC_IRQn					= 18,
  GPIOA_IRQn                = 19,
  GPIOB_IRQn                = 20,
  GPIOA0_GPIOB0_IRQn        = 21,
  GPIOA1_GPIOB1_IRQn		= 22,
  GPIOA2_GPIOB2_IRQn    	= 23,
  GPIOA3_GPIOB3_IRQn    	= 24,
  GPIOA4_GPIOB4_IRQn    	= 25,
  GPIOA5_GPIOB5_IRQn    	= 26,
  GPIOA6_GPIOB6_IRQn    	= 27,
  GPIOA7_GPIOB7_IRQn    	= 28,
  GPIOA8_GPIOB8_IRQn    	= 29,
  GPIOA9_GPIOB9_IRQn    	= 30,
  GPIOA10_GPIOB10_IRQn    	= 31,
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT		    0	   /*!< UART does not provide a MPU present or not	     */
#define __NVIC_PRIO_BITS		2	   /*!< UART Supports 2 Bits for the Priority Levels	 */
#define __Vendor_SysTickConfig  0	   /*!< Set to 1 if different SysTick Config is used	 */

#if   defined ( __CC_ARM )
  #pragma anon_unions
#endif

#include <stdio.h>
#include <stdbool.h>
#include "core_cm0.h"				   /* Cortex-M0 processor and core peripherals		     */
#include "system_SWM231.h"


/******************************************************************************/
/*				Device Specific Peripheral registers structures			 */
/******************************************************************************/
typedef struct {
	__IO uint32_t CLKSEL;				    //Clock Select

	__IO uint32_t CLKDIVx_ON;				//[0] CLK_DIVxʱ��Դ����

	__IO uint32_t CLKEN0;					//Clock Enable
	
		 uint32_t RESERVED;

	__IO uint32_t SLEEPCR;
	
		 uint32_t RESERVED2[3];
	
	__IO uint32_t RSTCR;
	__IO uint32_t RSTSR;					//Reset Status
		
		 uint32_t RESERVED3[22];
	
	__I  uint32_t CHIPID[4];
	
	__IO uint32_t BACKUP[6];				//Data Backup Register
	
		 uint32_t RESERVED4[22];
		 
	__IO uint32_t PAWKEN;				    //PORTA Wakeup Enable
	__IO uint32_t PBWKEN;	
	
		 uint32_t RESERVED5[10];

	__IO uint32_t PAWKSR;				    //PORTA Wakeup Status��д1����
	__IO uint32_t PBWKSR;	
	
		 uint32_t RESERVED6[(0x400-0x134)/4-1];
	
	__IO uint32_t IOFILT0;					//IO Filter 0
	__IO uint32_t IOFILT1;
	
		 uint32_t RESERVED7[(0x720-0x404)/4-1];
	
	__IO uint32_t PRSTEN;					//���踴λʹ�ܣ�ֻ�е�PRSTEN��ֵΪ0x55ʱ������дPRSTR0��PRSTR1
	__IO uint32_t PRSTR0;

    //Analog Control: 0x40044800
         uint32_t RESERVED8[(0x40044800-0x40000724)/4-1];
	
	__IO uint32_t PMUCR;
	
	__IO uint32_t VRFCR;					//Vref Control Register
	
	__IO uint32_t RCCR;						//RC Control Register
	
		 uint32_t RESERVED9;
    
	__IO uint32_t XTALCR;
	__IO uint32_t XTALSR;
		
		 uint32_t RESERVED10[2];
	
	__IO uint32_t PVDCR;
	__IO uint32_t PVDSR;
	
	__IO uint32_t LVRCR;
	
    __IO uint32_t ACMP0CR;					//Analog Comparator 0 Control Register
	__IO uint32_t ACMP1CR;
	__IO uint32_t ACMPCR;
	__IO uint32_t ACMPSR;					//Analog Comparator Status Register
	
	__IO uint32_t PGA0CR;					//PGA0 Control Register
	__IO uint32_t PGA1CR;
		 uint32_t RESERVED11;
	__IO uint32_t PGAREF;					//PGA Vref Control Register
	
	__IO uint32_t TEMPCR;					//Temperature Sensor Control Register
	
	__IO uint32_t ADCREF;					//ADC Vref select
} SYS_TypeDef;


#define SYS_CLKSEL_SYS_Pos			0		//ϵͳʱ��ѡ��	1 HRC	0 CLK_DIVx
#define SYS_CLKSEL_SYS_Msk			(0x01 << SYS_CLKSEL_SYS_Pos)
#define SYS_CLKSEL_CLK_DIVx_Pos		1		//ѡ��CLK_DIVx  0 CLK_DIV1   1 CLK_DIV2   2 CLK_DIV4   3 CLK_DIV8
#define SYS_CLKSEL_CLK_DIVx_Msk		(0x03 << SYS_CLKSEL_CLK_DIVx_Pos)
#define SYS_CLKSEL_CLK_Pos			3		//Clock Source  0 LRC   2 XTAL   3 HRC
#define SYS_CLKSEL_CLK_Msk			(0x03 << SYS_CLKSEL_CLK_Pos)
#define SYS_CLKSEL_IOFILT_Pos		5		//IO Filterʱ��ѡ��0 HRC   2 XTAL   3 LRC
#define SYS_CLKSEL_IOFILT_Msk		(0x03 << SYS_CLKSEL_IOFILT_Pos)
#define SYS_CLKSEL_WDT_Pos			7		//���Ź�ʱ��ѡ��  0 HRC   1 XTAL   2 LRC
#define SYS_CLKSEL_WDT_Msk			(0x03 << SYS_CLKSEL_WDT_Pos)

#define SYS_CLKDIV_ON_Pos           0
#define SYS_CLKDIV_ON_Msk           (0x01 << SYS_CLKDIV_ON_Pos)

#define SYS_CLKEN0_GPIOA_Pos		0
#define SYS_CLKEN0_GPIOA_Msk		(0x01 << SYS_CLKEN0_GPIOA_Pos)
#define SYS_CLKEN0_GPIOB_Pos		1
#define SYS_CLKEN0_GPIOB_Msk		(0x01 << SYS_CLKEN0_GPIOB_Pos)
#define SYS_CLKEN0_UART0_Pos		2
#define SYS_CLKEN0_UART0_Msk		(0x01 << SYS_CLKEN0_UART0_Pos)
#define SYS_CLKEN0_UART1_Pos		3
#define SYS_CLKEN0_UART1_Msk		(0x01 << SYS_CLKEN0_UART1_Pos)
#define SYS_CLKEN0_SPI0_Pos			4
#define SYS_CLKEN0_SPI0_Msk			(0x01 << SYS_CLKEN0_SPI0_Pos)
#define SYS_CLKEN0_TIMR_Pos			5
#define SYS_CLKEN0_TIMR_Msk			(0x01 << SYS_CLKEN0_TIMR_Pos)
#define SYS_CLKEN0_BTIMR_Pos		6
#define SYS_CLKEN0_BTIMR_Msk		(0x01 << SYS_CLKEN0_BTIMR_Pos)
#define SYS_CLKEN0_PWM_Pos			7
#define SYS_CLKEN0_PWM_Msk			(0x01 << SYS_CLKEN0_PWM_Pos)
#define SYS_CLKEN0_DIV_Pos			8
#define SYS_CLKEN0_DIV_Msk			(0x01 << SYS_CLKEN0_DIV_Pos)
#define SYS_CLKEN0_ANAC_Pos			9		//ģ����Ƶ�Ԫʱ��ʹ��
#define SYS_CLKEN0_ANAC_Msk			(0x01 << SYS_CLKEN0_ANAC_Pos)
#define SYS_CLKEN0_ADC0_Pos			10
#define SYS_CLKEN0_ADC0_Msk			(0x01 << SYS_CLKEN0_ADC0_Pos)
#define SYS_CLKEN0_IOFILT_Pos		11
#define SYS_CLKEN0_IOFILT_Msk		(0x01 << SYS_CLKEN0_IOFILT_Pos)
#define SYS_CLKEN0_WDT_Pos			12
#define SYS_CLKEN0_WDT_Msk			(0x01 << SYS_CLKEN0_WDT_Pos)
#define SYS_CLKEN0_FOC_Pos			13
#define SYS_CLKEN0_FOC_Msk			(0x01 << SYS_CLKEN0_FOC_Pos)

#define SYS_SLEEPCR_FLASH_Pos		1		//flash state when sleep, 0 standby, 1 powerdown
#define SYS_SLEEPCR_FLASH_Msk		(0x01 << SYS_SLEEPCR_FLASH_Pos)
#define SYS_SLEEPCR_WKEDGE_Pos		2		//wake-up edge, 0 falling edge, 1 rising edge
#define SYS_SLEEPCR_WKEDGE_Msk		(0x01 << SYS_SLEEPCR_WKEDGE_Pos)
#define SYS_SLEEPCR_DISDMA_Pos		3		//disable DMA when sleep, 0 DMA work, 1 DMA not work
#define SYS_SLEEPCR_DISDMA_Msk		(0x01 << SYS_SLEEPCR_DISDMA_Pos)
#define SYS_SLEEPCR_DISLDO_Pos		4		//disable LDO when sleep, 0 main LDO work, 1 main LDO close, LDO_LP work
#define SYS_SLEEPCR_DISLDO_Msk		(0x01 << SYS_SLEEPCR_DISLDO_Pos)

#define SYS_RSTCR_GPIO_Pos			0		//1 ��λ�������� GPIO
#define SYS_RSTCR_GPIO_Msk			(0x01 << SYS_RSTCR_GPIO_Pos)
#define SYS_RSTCR_KEY_Pos			16		//Ϊ 0x5A5A ʱ���ԸüĴ�����д������Ч
#define SYS_RSTCR_KEY_Msk			(0xFFFF << SYS_RSTCR_KEY_Pos)

#define SYS_RSTSR_POR_Pos			0		//1 ���ֹ�POR/LVR��λ��д1����
#define SYS_RSTSR_POR_Msk			(0x01 << SYS_RSTSR_POR_Pos)
#define SYS_RSTSR_WDT_Pos			1		//1 ���ֹ�WDT��λ��д1����
#define SYS_RSTSR_WDT_Msk			(0x01 << SYS_RSTSR_WDT_Pos)
#define SYS_RSTSR_PIN_Pos			2		//1 ���ֹ�NRST���Ÿ�λ��д1����
#define SYS_RSTSR_PIN_Msk			(0x01 << SYS_RSTSR_PIN_Pos)

#define SYS_IOFILT_TIM_Pos			0		//�˲�����ʱ�� = Tfilter_clk * ʱ�ӷ�Ƶ * 2^TIM
#define SYS_IOFILT_TIM_Msk			(0x0F << SYS_IOFILT_TIM_Pos)
#define SYS_IOFILT_CLKDIV_Pos		4		//0 ʱ�Ӳ���Ƶ   1 ʱ��32��Ƶ
#define SYS_IOFILT_CLKDIV_Msk		(0x01 << SYS_IOFILT_CLKDIV_Pos)
#define SYS_IOFILT_IO0EN_Pos		5		//IO0 �˲�ʹ��
#define SYS_IOFILT_IO0EN_Msk		(0x01 << SYS_IOFILT_IO0EN_Pos)
#define SYS_IOFILT_IO1EN_Pos		6
#define SYS_IOFILT_IO1EN_Msk		(0x01 << SYS_IOFILT_IO1EN_Pos)
#define SYS_IOFILT_IO2EN_Pos		7
#define SYS_IOFILT_IO2EN_Msk		(0x01 << SYS_IOFILT_IO2EN_Pos)
#define SYS_IOFILT_IO3EN_Pos		8
#define SYS_IOFILT_IO3EN_Msk		(0x01 << SYS_IOFILT_IO3EN_Pos)

#define SYS_PRSTR0_GPIOA_Pos		0		//1 ��λGPIOA    0 ����λ
#define SYS_PRSTR0_GPIOA_Msk		(0x01 <<SYS_PRSTR0_GPIOA_Pos)
#define SYS_PRSTR0_GPIOB_Pos		1
#define SYS_PRSTR0_GPIOB_Msk		(0x01 <<SYS_PRSTR0_GPIOB_Pos)
#define SYS_PRSTR0_UART0_Pos		2
#define SYS_PRSTR0_UART0_Msk		(0x01 <<SYS_PRSTR0_UART0_Pos)
#define SYS_PRSTR0_UART1_Pos		3
#define SYS_PRSTR0_UART1_Msk		(0x01 <<SYS_PRSTR0_UART1_Pos)
#define SYS_PRSTR0_SPI0_Pos			4
#define SYS_PRSTR0_SPI0_Msk			(0x01 <<SYS_PRSTR0_SPI0_Pos)
#define SYS_PRSTR0_TIMR_Pos			5
#define SYS_PRSTR0_TIMR_Msk			(0x01 <<SYS_PRSTR0_TIMR_Pos)
#define SYS_PRSTR0_BTIMR_Pos		6
#define SYS_PRSTR0_BTIMR_Msk		(0x01 <<SYS_PRSTR0_BTIMR_Pos)
#define SYS_PRSTR0_PWM_Pos			7
#define SYS_PRSTR0_PWM_Msk			(0x01 <<SYS_PRSTR0_PWM_Pos)
#define SYS_PRSTR0_DIV_Pos			8
#define SYS_PRSTR0_DIV_Msk			(0x01 <<SYS_PRSTR0_DIV_Pos)
#define SYS_PRSTR0_ANAC_Pos			9
#define SYS_PRSTR0_ANAC_Msk			(0x01 <<SYS_PRSTR0_ANAC_Pos)
#define SYS_PRSTR0_ADC0_Pos			10
#define SYS_PRSTR0_ADC0_Msk			(0x01 <<SYS_PRSTR0_ADC0_Pos)
#define SYS_PRSTR0_IOFILT_Pos		11
#define SYS_PRSTR0_IOFILT_Msk		(0x01 <<SYS_PRSTR0_IOFILT_Pos)
#define SYS_PRSTR0_WDT_Pos			12
#define SYS_PRSTR0_WDT_Msk			(0x01 <<SYS_PRSTR0_WDT_Pos)
#define SYS_PRSTR0_FOC_Pos			13
#define SYS_PRSTR0_FOC_Msk			(0x01 <<SYS_PRSTR0_FOC_Pos)

#define SYS_VRFCR_EN_Pos			0
#define SYS_VRFCR_EN_Msk			(0x01 << SYS_VRFCR_EN_Pos)
#define SYS_VRFCR_LVL_Pos			1		//0 2.4V   1 3.6V   2 4.5V
#define SYS_VRFCR_LVL_Msk			(0x03 << SYS_VRFCR_LVL_Pos)

#define SYS_RCCR_HON_Pos			0		//High speed RC ON
#define SYS_RCCR_HON_Msk			(0x01 << SYS_RCCR_HON_Pos)
#define SYS_RCCR_LON_Pos			1		//Low speed RC ON
#define SYS_RCCR_LON_Msk			(0x03 << SYS_RCCR_LON_Pos)

#define SYS_XTALCR_ON_Pos			0		//XTAL On
#define SYS_XTALCR_ON_Msk			(0x01 << SYS_XTALCR_ON_Pos)
#define SYS_XTALCR_BYPASS_Pos		1		//��Ƶ����ʱ��ֱ��
#define SYS_XTALCR_BYPASS_Msk		(0x01 << SYS_XTALCR_BYPASS_Pos)
#define SYS_XTALCR_DET_Pos			2		//XTAL Stop Detect Enable
#define SYS_XTALCR_DET_Msk			(0x01 << SYS_XTALCR_DET_Pos)

#define SYS_XTALSR_STOP_Pos			0		//XTAL Stop��д1����
#define SYS_XTALSR_STOP_Msk			(0x01 << SYS_XTALSR_STOP_Pos)

#define SYS_PVDCR_EN_Pos		    0		//PVD Enable
#define SYS_PVDCR_EN_Msk		    (0x01 << SYS_PVDCR_EN_Pos)
#define SYS_PVDCR_LVL_Pos			1		//PVD������ƽ��0 2.0v   1 2.3v   2 2.7v   3 3.0v   4 3.7v   5 4.0v   6 4.3v
#define SYS_PVDCR_LVL_Msk			(0x07 << SYS_PVDCR_LVL_Pos)
#define SYS_PVDCR_IE_Pos			4		//PVD Interrupt Enable
#define SYS_PVDCR_IE_Msk			(0x01 << SYS_PVDCR_IE_Pos)

#define SYS_PVDSR_ST_Pos			0		//PVD Status
#define SYS_PVDSR_ST_Msk			(0x01 << SYS_PVDSR_ST_Pos)
#define SYS_PVDSR_IF_Pos			1		//�жϱ�־��д1����
#define SYS_PVDSR_IF_Msk			(0x01 << SYS_PVDSR_IF_Pos)

#define SYS_LVRCR_EN_Pos			0		//LVR Enable
#define SYS_LVRCR_EN_Msk			(0x01 << SYS_LVRCR_EN_Pos)
#define SYS_LVRCR_LVL_Pos			1		//LVR������ƽ��0 1.8v   1 2.0v   2 2.5v   3 3.5v
#define SYS_LVRCR_LVL_Msk			(0x03 << SYS_LVRCR_LVL_Pos)
#define SYS_LVRCR_WEN_Pos			3		//LVRCR дʹ�ܣ�д LVRCR ʱ��λ������ 1
#define SYS_LVRCR_WEN_Msk			(0x01 << SYS_LVRCR_WEN_Pos)

#define SYS_ACMP0CR_EN_Pos			0
#define SYS_ACMP0CR_EN_Msk			(0x01 << SYS_ACMP0CR_EN_Pos)
#define SYS_ACMP0CR_HYS_Pos			1		//���͵�ѹ��0 1mV   1 10mV   2 20mV   3 50mV
#define SYS_ACMP0CR_HYS_Msk			(0x03 << SYS_ACMP0CR_HYS_Pos)
#define SYS_ACMP0CR_VNSEL_Pos		3		//����ѡ��0 VN   1 DAC_OUT   2 VPX
#define SYS_ACMP0CR_VNSEL_Msk		(0x03 << SYS_ACMP0CR_VNSEL_Pos)
#define SYS_ACMP0CR_VPSEL_Pos		5		//����ѡ��0 VP0   1 VP1   2 VP2   3 PGA0_VP   4 PGA0_IN
#define SYS_ACMP0CR_VPSEL_Msk		(0x07 << SYS_ACMP0CR_VPSEL_Pos)
#define SYS_ACMP0CR_VPXEN_Pos		8		//1 VP0/VP1/VP2��������,���ĵ���ΪVPX
#define SYS_ACMP0CR_VPXEN_Msk		(0x01 << SYS_ACMP0CR_VPXEN_Pos)
#define SYS_ACMP0CR_IE_Pos			16		//�ж�ʹ��
#define SYS_ACMP0CR_IE_Msk			(0x01 << SYS_ACMP0CR_IE_Pos)
#define SYS_ACMP0CR_TOPWM_Pos		17		//�Ƚ��������Ϊ PWM ɲ���ź�
#define SYS_ACMP0CR_TOPWM_Msk		(0x01 << SYS_ACMP0CR_TOPWM_Pos)

#define SYS_ACMP1CR_EN_Pos			0
#define SYS_ACMP1CR_EN_Msk			(0x01 << SYS_ACMP1CR_EN_Pos)
#define SYS_ACMP1CR_HYS_Pos			1
#define SYS_ACMP1CR_HYS_Msk			(0x03 << SYS_ACMP1CR_HYS_Pos)
#define SYS_ACMP1CR_VNSEL_Pos		3		//����ѡ��0 VN   1 DAC_OUT
#define SYS_ACMP1CR_VNSEL_Msk		(0x01 << SYS_ACMP1CR_VNSEL_Pos)
#define SYS_ACMP1CR_VPSEL_Pos		5		//����ѡ��0 VP   1 PGA1_VP   2 PGA0_OUT   3 PGA1_OUT   4 PGA1_IN
#define SYS_ACMP1CR_VPSEL_Msk		(0x07 << SYS_ACMP1CR_VPSEL_Pos)
#define SYS_ACMP1CR_IE_Pos			16
#define SYS_ACMP1CR_IE_Msk			(0x01 << SYS_ACMP1CR_IE_Pos)
#define SYS_ACMP1CR_TOPWM_Pos		17
#define SYS_ACMP1CR_TOPWM_Msk		(0x01 << SYS_ACMP1CR_TOPWM_Pos)

#define SYS_ACMPCR_DACEN_Pos		0 		//ACMP DACʹ��
#define SYS_ACMPCR_DACEN_Msk		(0x01 << SYS_ACMPCR_DACEN_Pos)
#define SYS_ACMPCR_DACVR_Pos		1 		//ACMP DAC�ο���ѹѡ��
#define SYS_ACMPCR_DACVR_Msk		(0x03 << SYS_ACMPCR_DACVR_Pos)
#define SYS_ACMPCR_DACDR_Pos		8 		//ACMP DAC���ݼĴ���
#define SYS_ACMPCR_DACDR_Msk		(0xFF << SYS_ACMPCR_DACDR_Pos)

#define SYS_ACMPSR_CMP0IF_Pos		0		//�жϱ�־��д1����
#define SYS_ACMPSR_CMP0IF_Msk		(0x01 << SYS_ACMPSR_CMP0IF_Pos)
#define SYS_ACMPSR_CMP1IF_Pos		1
#define SYS_ACMPSR_CMP1IF_Msk		(0x01 << SYS_ACMPSR_CMP1IF_Pos)
#define SYS_ACMPSR_CMP0OUT_Pos		8		//0 N > P   1 P > N
#define SYS_ACMPSR_CMP0OUT_Msk		(0x01 << SYS_ACMPSR_CMP0OUT_Pos)
#define SYS_ACMPSR_CMP1OUT_Pos		9
#define SYS_ACMPSR_CMP1OUT_Msk		(0x01 << SYS_ACMPSR_CMP1OUT_Pos)

#define SYS_PGA0CR_EN_Pos			0
#define SYS_PGA0CR_EN_Msk			(0x01 << SYS_PGA0CR_EN_Pos)
#define SYS_PGA0CR_MODE_Pos			1 	//0 OPA   1 PGA
#define SYS_PGA0CR_MODE_Msk			(0x01 << SYS_PGA0CR_MODE_Pos)
#define SYS_PGA0CR_ROUT_Pos			2 	//�������ѡ��0 open   1 100   2 1k   3 10k
#define SYS_PGA0CR_ROUT_Msk			(0x03 << SYS_PGA0CR_ROUT_Pos)
#define SYS_PGA0CR_GAIN_Pos			4 	//PGA ����ѡ��0 x1   1 x5   2 x10   3 x20
#define SYS_PGA0CR_GAIN_Msk			(0x03 << SYS_PGA0CR_GAIN_Pos)
#define SYS_PGA0CR_BUFEN_Pos		6 	//��� BUF ʹ��
#define SYS_PGA0CR_BUFEN_Msk		(0x01 << SYS_PGA0CR_BUFEN_Pos)
#define SYS_PGA0CR_BYPASS_Pos		7 	//��� BUF ��·
#define SYS_PGA0CR_BYPASS_Msk		(0x01 << SYS_PGA0CR_BYPASS_Pos)
#define SYS_PGA0CR_INSEL_Pos		8	//0 OPA0_IN, 1 OPA1_IN, 2 GND
#define SYS_PGA0CR_INSEL_Msk		(0x03 << SYS_PGA0CR_INSEL_Pos)

#define SYS_PGA1CR_EN_Pos			0
#define SYS_PGA1CR_EN_Msk			(0x01 << SYS_PGA1CR_EN_Pos)
#define SYS_PGA1CR_MODE_Pos			1
#define SYS_PGA1CR_MODE_Msk			(0x01 << SYS_PGA1CR_MODE_Pos)
#define SYS_PGA1CR_ROUT_Pos			2
#define SYS_PGA1CR_ROUT_Msk			(0x03 << SYS_PGA1CR_ROUT_Pos)
#define SYS_PGA1CR_GAIN_Pos			4
#define SYS_PGA1CR_GAIN_Msk			(0x03 << SYS_PGA1CR_GAIN_Pos)
#define SYS_PGA1CR_BUFEN_Pos		6
#define SYS_PGA1CR_BUFEN_Msk		(0x01 << SYS_PGA1CR_BUFEN_Pos)
#define SYS_PGA1CR_BYPASS_Pos		7
#define SYS_PGA1CR_BYPASS_Msk		(0x01 << SYS_PGA1CR_BYPASS_Pos)

#define SYS_PGAREF_REFSEL_Pos		0 		//PGA �ο���ѹѡ��0 1.2v   1 1.8v   2 2.25v   3 ADCVREF/2
#define SYS_PGAREF_REFSEL_Msk		(0x03 << SYS_PGAREF_REFSEL_Pos)

#define SYS_TEMPCR_EN_Pos			0
#define SYS_TEMPCR_EN_Msk			(0x01 << SYS_TEMPCR_EN_Pos)

#define SYS_ADCREF_REFSEL_Pos		0 		//ADC �ο���ѹѡ��0 VDD   1 VREF
#define SYS_ADCREF_REFSEL_Msk		(0x01 << SYS_ADCREF_REFSEL_Pos)




typedef struct {
	__IO uint32_t FUNC0;					//���Ź���ѡ��
	
	__IO uint32_t FUNC1;
	
		 uint32_t RESERVED[62];
	
    __IO uint32_t PULLU;              		//����ʹ��
    
         uint32_t RESERVED2[63];
    
    __IO uint32_t PULLD;	              	//����ʹ��
    
         uint32_t RESERVED3[63];
    
    __IO uint32_t INEN;               		//����ʹ��
    
         uint32_t RESERVED4[63];

	__IO uint32_t OPEND;              		//��©ʹ��
} PORT_TypeDef;




typedef struct {
	__IO uint32_t ODR;
#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7
#define PIN8    8
#define PIN9    9
#define PIN10   10
#define PIN11   11
#define PIN12   12
#define PIN13   13
#define PIN14   14
#define PIN15   15

	__IO uint32_t DIR;					    //0 ����	1 ���

	__IO uint32_t INTLVLTRG;				//Interrupt Level Trigger  1 ��ƽ�����ж�	0 ���ش����ж�

	__IO uint32_t INTBE;					//Both Edge����INTLVLTRG��Ϊ���ش����ж�ʱ����λ��1��ʾ�����غ��½��ض������жϣ���0ʱ����������INTRISEENѡ��

	__IO uint32_t INTRISEEN;				//Interrupt Rise Edge Enable   1 ������/�ߵ�ƽ�����ж�	0 �½���/�͵�ƽ�����ж�

	__IO uint32_t INTEN;					//1 �ж�ʹ��	0 �жϽ�ֹ

	__I  uint32_t INTRAWSTAT;			    //�жϼ�ⵥԪ�Ƿ��⵽�˴����жϵ����� 1 ��⵽���жϴ�������	0 û�м�⵽�жϴ�������

	__I  uint32_t INTSTAT;				    //INTSTAT.PIN0 = INTRAWSTAT.PIN0 & INTEN.PIN0

	__O  uint32_t INTCLR;				    //д1����жϱ�־��ֻ�Ա��ش����ж�����
	
		 uint32_t RESERVED[3];
	
	__I  uint32_t IDR;
	
		 uint32_t RESERVED2[3];
	
	__IO uint32_t DATAPIN0;					//PIN0���ŵ�DATA�Ĵ������������Ŷ�Ӧ����32λ�Ĵ���������ʵ��ԭ��д����
	__IO uint32_t DATAPIN1;
	__IO uint32_t DATAPIN2;
	__IO uint32_t DATAPIN3;
	__IO uint32_t DATAPIN4;
	__IO uint32_t DATAPIN5;
	__IO uint32_t DATAPIN6;
	__IO uint32_t DATAPIN7;
	__IO uint32_t DATAPIN8;
	__IO uint32_t DATAPIN9;
	__IO uint32_t DATAPIN10;
	__IO uint32_t DATAPIN11;
	__IO uint32_t DATAPIN12;
	__IO uint32_t DATAPIN13;
	__IO uint32_t DATAPIN14;
	__IO uint32_t DATAPIN15;
} GPIO_TypeDef;




typedef struct {
	__IO uint32_t LOAD;						//��ʱ������ֵ��ʹ�ܺ�ʱ���Ӵ���ֵ��ʼ���µݼ�����

	__I  uint32_t VALUE;					//��ʱ����ǰֵ��LDVAL-CVAL �ɼ������ʱʱ��

	__IO uint32_t CR;
	
		 uint32_t RESERVED;
	
	__IO uint32_t IE;
		
	__IO uint32_t IF;
	
	__IO uint32_t HALT;						//[0] 1 ��ͣ����    0 �ָ�����
	
	__IO uint32_t OCCR;
	
	__IO uint32_t OCMAT;
	__IO uint32_t RESERVED2;
	
	__IO uint32_t ICLOW;
	__IO uint32_t ICHIGH;
	
	__IO uint32_t PREDIV;					//Ԥ��Ƶ��8λ
} TIMR_TypeDef;


#define TIMR_LOAD_VALUE_Pos			0
#define TIMR_LOAD_VALUE_Msk			(0xFFFFFF << TIMR_LOAD_VALUE_Pos)
#define TIMR_LOAD_RELOAD_Pos		24		//1 ��������������д���LOADֵ��ʼ������ֻ��BTIMR�д˹���
#define TIMR_LOAD_RELOAD_Msk		(0x01 << TIMR_LOAD_RELOAD_Pos)

#define TIMR_CR_CLKSRC_Pos			0		//ʱ��Դ��  0 �ڲ�ϵͳʱ��	2 �ⲿ�����������
#define TIMR_CR_CLKSRC_Msk			(0x03 << TIMR_CR_CLKSRC_Pos)
#define TIMR_CR_MODE_Pos			2		//����ģʽ��0 ��ʱ��    1 ���벶��    2 ����Ƚ�
#define TIMR_CR_MODE_Msk			(0x03 << TIMR_CR_MODE_Pos)
#define TIMR_CR_ICEDGE_Pos			4		//���벶��ģʽ�¼����������أ�0 ˫����   1 ������   2 �½���
#define TIMR_CR_ICEDGE_Msk			(0x03 << TIMR_CR_ICEDGE_Pos)

#define TIMR_IE_TO_Pos				0		//Time out
#define TIMR_IE_TO_Msk				(0x01 << TIMR_IE_TO_Pos)
#define TIMR_IE_OC0_Pos				1		//����Ƚϣ���һ����ת��
#define TIMR_IE_OC0_Msk				(0x01 << TIMR_IE_OC0_Pos)
#define TIMR_IE_OC1_Pos				2		//����Ƚϣ��ڶ�����ת��
#define TIMR_IE_OC1_Msk				(0x01 << TIMR_IE_OC1_Pos)
#define TIMR_IE_ICR_Pos				3		//���벶���������ж�
#define TIMR_IE_ICR_Msk				(0x01 << TIMR_IE_ICR_Pos)
#define TIMR_IE_ICF_Pos				4		//���벶���½����ж�
#define TIMR_IE_ICF_Msk				(0x01 << TIMR_IE_ICF_Pos)

#define TIMR_IF_TO_Pos				0		//��ʱ�жϱ�־��д1����
#define TIMR_IF_TO_Msk				(0x01 << TIMR_IF_TO_Pos)
#define TIMR_IF_OC0_Pos				1
#define TIMR_IF_OC0_Msk				(0x01 << TIMR_IF_OC0_Pos)
#define TIMR_IF_OC1_Pos				2
#define TIMR_IF_OC1_Msk				(0x01 << TIMR_IF_OC1_Pos)
#define TIMR_IF_ICR_Pos				3
#define TIMR_IF_ICR_Msk				(0x01 << TIMR_IF_ICR_Pos)
#define TIMR_IF_ICF_Pos				4
#define TIMR_IF_ICF_Msk				(0x01 << TIMR_IF_ICF_Pos)

#define TIMR_OCCR_FORCELVL_Pos		0		//Force Levle��ǿ�������ƽ
#define TIMR_OCCR_FORCELVL_Msk		(0x01 << TIMR_OCCR_FORCELVL_Pos)
#define TIMR_OCCR_INITLVL_Pos		1		//Initial Level, ��ʼ�����ƽ��Timerֹͣʱ����ģʽ���ǡ�����Ƚϡ�ʱ�������ƽ
#define TIMR_OCCR_INITLVL_Msk		(0x01 << TIMR_OCCR_INITLVL_Pos)
#define TIMR_OCCR_FORCEEN_Pos		2		//Force Enable, ǿ�����ʹ��
#define TIMR_OCCR_FORCEEN_Msk		(0x01 << TIMR_OCCR_FORCEEN_Pos)


typedef struct {
		 uint32_t RSVD[8];
	
	__IO uint32_t ICSR;						//Input Capture Pin Status
	
		 uint32_t RSVD2[7];
	
	__IO uint32_t EN;
	
		 uint32_t RSVD3[3];
	
	__IO uint32_t HALLIE;					//[0] HALL�ж�ʹ��
	
	__IO uint32_t HALLIF;
	
	__IO uint32_t HALLEN;					//[0] HALL���ܿ���
	
	__IO uint32_t HALLDR;					//HALL���������ؽ�������������ֵ - ��ǰֵ������˼Ĵ���
	
		 uint32_t RSVD4;
	
	__IO uint32_t HALLSR;
} TIMRG_TypeDef;


#define TIMRG_HALLIF_IN0_Pos		0		//HALL�����ź�0�����жϱ�־��д1����
#define TIMRG_HALLIF_IN0_Msk		(0x01 << TIMRG_HALLIF_IN0_Pos)
#define TIMRG_HALLIF_IN1_Pos		1
#define TIMRG_HALLIF_IN1_Msk		(0x01 << TIMRG_HALLIF_IN1_Pos)
#define TIMRG_HALLIF_IN2_Pos		2
#define TIMRG_HALLIF_IN2_Msk		(0x01 << TIMRG_HALLIF_IN2_Pos)

#define TIMRG_HALLSR_IN0_Pos		0		//HALL�����źŵ�ǰ��ƽ
#define TIMRG_HALLSR_IN0_Msk		(0x01 << TIMRG_HALLSR_IN0_Pos)
#define TIMRG_HALLSR_IN1_Pos		1
#define TIMRG_HALLSR_IN1_Msk		(0x01 << TIMRG_HALLSR_IN1_Pos)
#define TIMRG_HALLSR_IN2_Pos		2
#define TIMRG_HALLSR_IN2_Msk		(0x01 << TIMRG_HALLSR_IN2_Pos)

#define TIMRG_ICSR_TIMR0_Pos		0
#define TIMRG_ICSR_TIMR0_Msk		(0x01 << TIMRG_ICSR_TIMR0_Pos)
#define TIMRG_ICSR_TIMR1_Pos		1
#define TIMRG_ICSR_TIMR1_Msk		(0x01 << TIMRG_ICSR_TIMR1_Pos)
#define TIMRG_ICSR_TIMR2_Pos		2
#define TIMRG_ICSR_TIMR2_Msk		(0x01 << TIMRG_ICSR_TIMR2_Pos)

#define TIMRG_EN_TIMR0_Pos			0
#define TIMRG_EN_TIMR0_Msk			(0x01 << TIMRG_EN_TIMR0_Pos)
#define TIMRG_EN_TIMR1_Pos			1
#define TIMRG_EN_TIMR1_Msk			(0x01 << TIMRG_EN_TIMR1_Pos)
#define TIMRG_EN_TIMR2_Pos			2
#define TIMRG_EN_TIMR2_Msk			(0x01 << TIMRG_EN_TIMR2_Pos)
#define TIMRG_EN_TIMR3_Pos			3
#define TIMRG_EN_TIMR3_Msk			(0x01 << TIMRG_EN_TIMR3_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t CTRL;
	
	__IO uint32_t BAUD;
	
	__IO uint32_t FIFO;
	
	__IO uint32_t LINCR;
	
	union {
		__IO uint32_t CTSCR;
		
		__IO uint32_t RTSCR;
	};
	
	__IO uint32_t CFG;
	
	__IO uint32_t TOCR;						//Timeout Control Register
} UART_TypeDef;


#define UART_DATA_DATA_Pos			0
#define UART_DATA_DATA_Msk			(0x1FF << UART_DATA_DATA_Pos)
#define UART_DATA_VALID_Pos			9		//��DATA�ֶ�����Ч�Ľ�������ʱ����λӲ����1����ȡ���ݺ��Զ�����
#define UART_DATA_VALID_Msk			(0x01 << UART_DATA_VALID_Pos)
#define UART_DATA_PAERR_Pos			10		//Parity Error
#define UART_DATA_PAERR_Msk			(0x01 << UART_DATA_PAERR_Pos)

#define UART_CTRL_TXIDLE_Pos		0		//TX IDLE: 0 ���ڷ�������	1 ����״̬��û�����ݷ���
#define UART_CTRL_TXIDLE_Msk		(0x01 << UART_CTRL_TXIDLE_Pos)
#define UART_CTRL_TXFF_Pos		    1		//TX FIFO Full
#define UART_CTRL_TXFF_Msk		    (0x01 << UART_CTRL_TXFF_Pos)
#define UART_CTRL_TXIE_Pos			2		//TX �ж�ʹ��: 1 TX FF �����������趨����ʱ�����ж�
#define UART_CTRL_TXIE_Msk			(0x01 << UART_CTRL_TXIE_Pos)
#define UART_CTRL_RXNE_Pos			3		//RX FIFO Not Empty
#define UART_CTRL_RXNE_Msk			(0x01 << UART_CTRL_RXNE_Pos)
#define UART_CTRL_RXIE_Pos			4		//RX �ж�ʹ��: 1 RX FF �����ݴﵽ�趨����ʱ�����ж�
#define UART_CTRL_RXIE_Msk			(0x01 << UART_CTRL_RXIE_Pos)
#define UART_CTRL_RXOV_Pos			5		//RX FIFO Overflow��д1����
#define UART_CTRL_RXOV_Msk			(0x01 << UART_CTRL_RXOV_Pos)
#define UART_CTRL_TXDOIE_Pos		6		//TX Done �ж�ʹ�ܣ�����FIFO���ҷ��ͷ�����λ�Ĵ����ѽ����һλ���ͳ�ȥ
#define UART_CTRL_TXDOIE_Msk		(0x01 << UART_CTRL_TXDOIE_Pos)
#define UART_CTRL_EN_Pos			9
#define UART_CTRL_EN_Msk			(0x01 << UART_CTRL_EN_Pos)
#define UART_CTRL_LOOP_Pos			10
#define UART_CTRL_LOOP_Msk			(0x01 << UART_CTRL_LOOP_Pos)
#define UART_CTRL_TOIE_Pos			14		//TimeOut �ж�ʹ�ܣ����յ��ϸ��ַ��󣬳��� TOTIME/BAUDRAUD ��û�н��յ��µ�����
#define UART_CTRL_TOIE_Msk			(0x01 << UART_CTRL_TOIE_Pos)
#define UART_CTRL_DATA9b_Pos		18		//1 9λ����λ    0 8λ����λ
#define UART_CTRL_DATA9b_Msk		(0x01 << UART_CTRL_DATA9b_Pos)
#define UART_CTRL_PARITY_Pos		19		//000 ��У��    001 ��У��   011 żУ��   101 �̶�Ϊ1    111 �̶�Ϊ0
#define UART_CTRL_PARITY_Msk		(0x07 << UART_CTRL_PARITY_Pos)
#define UART_CTRL_STOP2b_Pos		22		//1 2λֹͣλ    0 1λֹͣλ
#define UART_CTRL_STOP2b_Msk		(0x03 << UART_CTRL_STOP2b_Pos)

#define UART_BAUD_BAUD_Pos			0		//���ڲ����� = SYS_Freq/16/BAUD - 1
#define UART_BAUD_BAUD_Msk			(0x3FFF << UART_BAUD_BAUD_Pos)
#define UART_BAUD_TXD_Pos			14		//ͨ����λ��ֱ�Ӷ�ȡ����TXD�����ϵĵ�ƽ
#define UART_BAUD_TXD_Msk			(0x01 << UART_BAUD_TXD_Pos)
#define UART_BAUD_RXD_Pos			15		//ͨ����λ��ֱ�Ӷ�ȡ����RXD�����ϵĵ�ƽ
#define UART_BAUD_RXD_Msk			(0x01 << UART_BAUD_RXD_Pos)
#define UART_BAUD_RXTOIF_Pos		16		//����&��ʱ���жϱ�־ = RXIF | TOIF
#define UART_BAUD_RXTOIF_Msk		(0x01 << UART_BAUD_RXTOIF_Pos)
#define UART_BAUD_TXIF_Pos			17		//�����жϱ�־ = TXTHRF & TXIE
#define UART_BAUD_TXIF_Msk			(0x01 << UART_BAUD_TXIF_Pos)
#define UART_BAUD_RXTHRF_Pos		19		//RX FIFO Threshold Flag��RX FIFO�����ݴﵽ�趨������RXLVL >  RXTHR��ʱӲ����1
#define UART_BAUD_RXTHRF_Msk		(0x01 << UART_BAUD_RXTHRF_Pos)
#define UART_BAUD_TXTHRF_Pos		20		//TX FIFO Threshold Flag��TX FIFO�����������趨������TXLVL <= TXTHR��ʱӲ����1
#define UART_BAUD_TXTHRF_Msk		(0x01 << UART_BAUD_TXTHRF_Pos)
#define UART_BAUD_TOIF_Pos			21		//TimeOut �жϱ�־������ TOTIME/BAUDRAUD ��û�н��յ��µ�����ʱ��TOIE=1����λ��Ӳ����λ
#define UART_BAUD_TOIF_Msk			(0x01 << UART_BAUD_TOIF_Pos)
#define UART_BAUD_RXIF_Pos			22		//�����жϱ�־ = RXTHRF & RXIE
#define UART_BAUD_RXIF_Msk			(0x01 << UART_BAUD_RXIF_Pos)
#define UART_BAUD_ABREN_Pos			23		//Auto Baudrate Enable��д1�����Զ�������У׼����ɺ��Զ�����
#define UART_BAUD_ABREN_Msk			(0x01 << UART_BAUD_ABREN_Pos)
#define UART_BAUD_ABRBIT_Pos		24		//Auto Baudrate Bit�����ڼ��㲨���ʵļ��λ����0 1λ��ͨ������ʼλ           ������㲨���ʣ�Ҫ���Ͷ˷���0xFF
											//                                             1 2λ��ͨ������ʼλ��1λ����λ������㲨���ʣ�Ҫ���Ͷ˷���0xFE
											//                                             1 4λ��ͨ������ʼλ��3λ����λ������㲨���ʣ�Ҫ���Ͷ˷���0xF8
											//                                             1 8λ��ͨ������ʼλ��7λ����λ������㲨���ʣ�Ҫ���Ͷ˷���0x80
#define UART_BAUD_ABRBIT_Msk		(0x03 << UART_BAUD_ABRBIT_Pos)
#define UART_BAUD_ABRERR_Pos		26		//Auto Baudrate Error��0 �Զ�������У׼�ɹ�     1 �Զ�������У׼ʧ��
#define UART_BAUD_ABRERR_Msk		(0x01 << UART_BAUD_ABRERR_Pos)
#define UART_BAUD_TXDOIF_Pos		27		//TX Done �жϱ�־������FIFO���ҷ��ͷ�����λ�Ĵ����ѽ����һλ���ͳ�ȥ
#define UART_BAUD_TXDOIF_Msk		(0x01 << UART_BAUD_TXDOIF_Pos)
#define UART_BAUD_FRAC_Pos			28		//�����ʷ�ƵֵС������
#define UART_BAUD_FRAC_Msk			(0x0Fu << UART_BAUD_FRAC_Pos)

#define UART_FIFO_RXLVL_Pos			0		//RX FIFO Level��RX FIFO ���ַ�����
#define UART_FIFO_RXLVL_Msk			(0xFF << UART_FIFO_RXLVL_Pos)
#define UART_FIFO_TXLVL_Pos			8		//TX FIFO Level��TX FIFO ���ַ�����
#define UART_FIFO_TXLVL_Msk			(0xFF << UART_FIFO_TXLVL_Pos)
#define UART_FIFO_RXTHR_Pos			16		//RX FIFO Threshold��RX�жϴ������ޣ��ж�ʹ��ʱ RXLVL >  RXTHR ����RX�ж�
#define UART_FIFO_RXTHR_Msk			(0xFF << UART_FIFO_RXTHR_Pos)
#define UART_FIFO_TXTHR_Pos			24		//TX FIFO Threshold��TX�жϴ������ޣ��ж�ʹ��ʱ TXLVL <= TXTHR ����TX�ж�
#define UART_FIFO_TXTHR_Msk			(0xFFu<< UART_FIFO_TXTHR_Pos)

#define UART_LINCR_BRKDETIE_Pos		0		//��⵽LIN Break�ж�ʹ��
#define UART_LINCR_BRKDETIE_Msk		(0x01 << UART_LINCR_BRKDETIE_Pos)
#define UART_LINCR_BRKDETIF_Pos		1		//��⵽LIN Break�ж�״̬
#define UART_LINCR_BRKDETIF_Msk		(0x01 << UART_LINCR_BRKDETIF_Pos)
#define UART_LINCR_GENBRKIE_Pos		2		//����LIN Break����ж�ʹ��
#define UART_LINCR_GENBRKIE_Msk		(0x01 << UART_LINCR_GENBRKIE_Pos)
#define UART_LINCR_GENBRKIF_Pos		3		//����LIN Break����ж�״̬
#define UART_LINCR_GENBRKIF_Msk		(0x01 << UART_LINCR_GENBRKIF_Pos)
#define UART_LINCR_GENBRK_Pos		4		//����LIN Break����������Զ�����
#define UART_LINCR_GENBRK_Msk		(0x01 << UART_LINCR_GENBRK_Pos)

#define UART_CTSCR_EN_Pos			0		//CTS����ʹ��
#define UART_CTSCR_EN_Msk			(0x01 << UART_CTSCR_EN_Pos)
#define UART_CTSCR_POL_Pos			2		//CTS�źż��ԣ�0 ����Ч��CTS����Ϊ�ͱ�ʾ���Է�������
#define UART_CTSCR_POL_Msk			(0x01 << UART_CTSCR_POL_Pos)
#define UART_CTSCR_STAT_Pos			7		//CTS�źŵĵ�ǰ״̬
#define UART_CTSCR_STAT_Msk			(0x01 << UART_CTSCR_STAT_Pos)

#define UART_RTSCR_EN_Pos			1		//RTS����ʹ��
#define UART_RTSCR_EN_Msk			(0x01 << UART_RTSCR_EN_Pos)
#define UART_RTSCR_POL_Pos			3		//RTS�źż���    0 ����Ч��RTS����Ϊ�ͱ�ʾ���Խ�������
#define UART_RTSCR_POL_Msk			(0x01 << UART_RTSCR_POL_Pos)
#define UART_RTSCR_THR_Pos			4		//RTS���صĴ�����ֵ    0 1�ֽ�    1 2�ֽ�    2 4�ֽ�    3 6�ֽ�
#define UART_RTSCR_THR_Msk			(0x07 << UART_RTSCR_THR_Pos)
#define UART_RTSCR_STAT_Pos			8		//RTS�źŵĵ�ǰ״̬
#define UART_RTSCR_STAT_Msk			(0x01 << UART_RTSCR_STAT_Pos)

#define UART_CFG_MSBF_Pos			1		//���շ���MSB First
#define UART_CFG_MSBF_Msk			(0x01 << UART_CFG_MSBF_Pos)
#define UART_CFG_BRKTXLEN_Pos		2		//1��ʾ1bit���Դ����ƣ�Ĭ��ֵ13
#define UART_CFG_BRKTXLEN_Msk		(0x0F << UART_CFG_BRKTXLEN_Pos)
#define UART_CFG_BRKRXLEN_Pos		6		//0��ʾ1bit���Դ����ƣ�Ĭ��ֵ12
#define UART_CFG_BRKRXLEN_Msk		(0x0F << UART_CFG_BRKRXLEN_Pos)
#define UART_CFG_RXINV_Pos			10		//���յ�ƽ��ת
#define UART_CFG_RXINV_Msk			(0x01 << UART_CFG_RXINV_Pos)
#define UART_CFG_TXINV_Pos			11		//���͵�ƽ��ת
#define UART_CFG_TXINV_Msk			(0x01 << UART_CFG_TXINV_Pos)

#define UART_TOCR_TIME_Pos			0		//��ʱʱ�䳤�ȣ���λΪ 10/BAUDRATE ��
#define UART_TOCR_TIME_Msk			(0xFFF<< UART_TOCR_TIME_Pos)
#define UART_TOCR_MODE_Pos			12		//0 ֻ�е�FIFO������ʱ�Ŵ�����ʱ�ж�    1 ��ʹFIFO��û����Ҳ�ɴ�����ʱ�ж�
#define UART_TOCR_MODE_Msk			(0x01 << UART_TOCR_MODE_Pos)
#define UART_TOCR_IFCLR_Pos			13		//TO Interrupt Flag Clear��д1�����ʱ�жϱ�־
#define UART_TOCR_IFCLR_Msk			(0x01 << UART_TOCR_IFCLR_Pos)




typedef struct {
	__IO uint32_t CTRL;

	__IO uint32_t DATA;

	__IO uint32_t STAT;

	__IO uint32_t IE;

	__IO uint32_t IF;
} SPI_TypeDef;


#define SPI_CTRL_CLKDIV_Pos			0		//Clock Divider, SPI����ʱ�� = SYS_Freq/pow(2, CLKDIV+2)
#define SPI_CTRL_CLKDIV_Msk			(0x07 << SPI_CTRL_CLKDIV_Pos)
#define SPI_CTRL_EN_Pos				3
#define SPI_CTRL_EN_Msk				(0x01 << SPI_CTRL_EN_Pos)
#define SPI_CTRL_SIZE_Pos			4		//Data Size Select, ȡֵ3--15����ʾ4--16λ
#define SPI_CTRL_SIZE_Msk			(0x0F << SPI_CTRL_SIZE_Pos)
#define SPI_CTRL_CPHA_Pos			8		//0 ��SCLK�ĵ�һ�������ز�������	1 ��SCLK�ĵڶ��������ز�������
#define SPI_CTRL_CPHA_Msk			(0x01 << SPI_CTRL_CPHA_Pos)
#define SPI_CTRL_CPOL_Pos			9		//0 ����״̬��SCLKΪ�͵�ƽ		  1 ����״̬��SCLKΪ�ߵ�ƽ
#define SPI_CTRL_CPOL_Msk			(0x01 << SPI_CTRL_CPOL_Pos)
#define SPI_CTRL_FFS_Pos			10		//Frame Format Select, 0 SPI	1 TI SSI	2 I2S	3 SPI Flash
#define SPI_CTRL_FFS_Msk			(0x03 << SPI_CTRL_FFS_Pos)
#define SPI_CTRL_MSTR_Pos			12		//Master, 1 ��ģʽ	0 ��ģʽ
#define SPI_CTRL_MSTR_Msk			(0x01 << SPI_CTRL_MSTR_Pos)
#define SPI_CTRL_FAST_Pos			13		//1 SPI����ʱ�� = SYS_Freq/2    0 SPI����ʱ����SPI->CTRL.CLKDIV����
#define SPI_CTRL_FAST_Msk			(0x01 << SPI_CTRL_FAST_Pos)
#define SPI_CTRL_DMATXEN_Pos		14		//1 ͨ��DMAдFIFO    0 ͨ��MCUдFIFO
#define SPI_CTRL_DMATXEN_Msk		(0x01 << SPI_CTRL_DMATXEN_Pos)
#define SPI_CTRL_DMARXEN_Pos		15		//1 ͨ��DMA��FIFO    0 ͨ��MCU��FIFO
#define SPI_CTRL_DMARXEN_Msk		(0x01 << SPI_CTRL_DMARXEN_Pos)
#define SPI_CTRL_FILTE_Pos			16		//1 ��SPI�����źŽ���ȥ������    0 ��SPI�����źŲ�����ȥ������
#define SPI_CTRL_FILTE_Msk			(0x01 << SPI_CTRL_FILTE_Pos)
#define SPI_CTRL_SSN_H_Pos			17		//0 ���������SSNʼ��Ϊ0    	 1 ���������ÿ�ַ�֮��ὫSSN���߰��SCLK����
#define SPI_CTRL_SSN_H_Msk			(0x01 << SPI_CTRL_SSN_H_Pos)
#define SPI_CTRL_RFTHR_Pos			18		//RX FIFO Threshold��0 ����FIFO��������1������   ...   7 ����FIFO��������8������
#define SPI_CTRL_RFTHR_Msk			(0x07 << SPI_CTRL_RFTHR_Pos)
#define SPI_CTRL_TFTHR_Pos			21		//TX FIFO Threshold��0 ����FIFO��������0������   ...   7 ����FIFO��������7������
#define SPI_CTRL_TFTHR_Msk			(0x07 << SPI_CTRL_TFTHR_Pos)
#define SPI_CTRL_RFCLR_Pos			24		//RX FIFO Clear
#define SPI_CTRL_RFCLR_Msk			(0x01 << SPI_CTRL_RFCLR_Pos)
#define SPI_CTRL_TFCLR_Pos			25		//TX FIFO Clear
#define SPI_CTRL_TFCLR_Msk			(0x01 << SPI_CTRL_TFCLR_Pos)
#define SPI_CTRL_LSBF_Pos			28		//LSB Fisrt
#define SPI_CTRL_LSBF_Msk			(0x01 << SPI_CTRL_LSBF_Pos)
#define SPI_CTRL_NSYNC_Pos			29		//0 ��SPI�����źŽ��в���ͬ��    1 ��SPI�����źŲ����в���ͬ��
#define SPI_CTRL_NSYNC_Msk			(0x01 << SPI_CTRL_NSYNC_Pos)

#define SPI_STAT_WTC_Pos			0		//Word Transmit Complete��ÿ�������һ����������Ӳ����1�����д1����
#define SPI_STAT_WTC_Msk			(0x01 << SPI_STAT_WTC_Pos)
#define SPI_STAT_TFE_Pos			1		//����FIFO Empty
#define SPI_STAT_TFE_Msk			(0x01 << SPI_STAT_TFE_Pos)
#define SPI_STAT_TFNF_Pos			2		//����FIFO Not Full
#define SPI_STAT_TFNF_Msk			(0x01 << SPI_STAT_TFNF_Pos)
#define SPI_STAT_RFNE_Pos			3		//����FIFO Not Empty
#define SPI_STAT_RFNE_Msk			(0x01 << SPI_STAT_RFNE_Pos)
#define SPI_STAT_RFF_Pos			4		//����FIFO Full
#define SPI_STAT_RFF_Msk			(0x01 << SPI_STAT_RFF_Pos)
#define SPI_STAT_RFOV_Pos			5		//����FIFO Overflow
#define SPI_STAT_RFOV_Msk			(0x01 << SPI_STAT_RFOV_Pos)
#define SPI_STAT_TFLVL_Pos			6		//����FIFO�����ݸ����� 0 TFNF=0ʱ��ʾFIFO����8�����ݣ�TFNF=1ʱ��ʾFIFO����0������	1--7 FIFO����1--7������
#define SPI_STAT_TFLVL_Msk			(0x07 << SPI_STAT_TFLVL_Pos)
#define SPI_STAT_RFLVL_Pos			9		//����FIFO�����ݸ����� 0 RFF =1ʱ��ʾFIFO����8�����ݣ�RFF =0ʱ��ʾFIFO����0������	1--7 FIFO����1--7������
#define SPI_STAT_RFLVL_Msk			(0x07 << SPI_STAT_RFLVL_Pos)
#define SPI_STAT_BUSY_Pos			15
#define SPI_STAT_BUSY_Msk			(0x01 << SPI_STAT_BUSY_Pos)

#define SPI_IE_RFOV_Pos				0
#define SPI_IE_RFOV_Msk				(0x01 << SPI_IE_RFOV_Pos)
#define SPI_IE_RFF_Pos				1
#define SPI_IE_RFF_Msk				(0x01 << SPI_IE_RFF_Pos)
#define SPI_IE_RFHF_Pos				2
#define SPI_IE_RFHF_Msk				(0x01 << SPI_IE_RFHF_Pos)
#define SPI_IE_TFE_Pos				3
#define SPI_IE_TFE_Msk				(0x01 << SPI_IE_TFE_Pos)
#define SPI_IE_TFHF_Pos				4		//����FIFO�����ݸ�������4
#define SPI_IE_TFHF_Msk				(0x01 << SPI_IE_TFHF_Pos)
#define SPI_IE_RFTHR_Pos			5		//����FIFO�����ݸ�������CTRL.RFTHR�趨ֵ�ж�ʹ��
#define SPI_IE_RFTHR_Msk			(0x01 << SPI_IE_RFTHR_Pos)
#define SPI_IE_TFTHR_Pos			6		//����FIFO�����ݸ���С��CTRL.TFTHR�趨ֵ�ж�ʹ��
#define SPI_IE_TFTHR_Msk			(0x01 << SPI_IE_TFTHR_Pos)
#define SPI_IE_WTC_Pos				8		//Word Transmit Complete
#define SPI_IE_WTC_Msk				(0x01 << SPI_IE_WTC_Pos)
#define SPI_IE_FTC_Pos				9		//Frame Transmit Complete
#define SPI_IE_FTC_Msk				(0x01 << SPI_IE_FTC_Pos)
#define SPI_IE_CSFALL_Pos			10		//�ӻ�ģʽ�£�CS�½����ж�ʹ��
#define SPI_IE_CSFALL_Msk			(0x01 << SPI_IE_CSFALL_Pos)
#define SPI_IE_CSRISE_Pos			11		//�ӻ�ģʽ�£�CS�������ж�ʹ��
#define SPI_IE_CSRISE_Msk			(0x01 << SPI_IE_CSRISE_Pos)

#define SPI_IF_RFOV_Pos				0		//д1����
#define SPI_IF_RFOV_Msk				(0x01 << SPI_IF_RFOV_Pos)
#define SPI_IF_RFF_Pos				1		//д1����
#define SPI_IF_RFF_Msk				(0x01 << SPI_IF_RFF_Pos)
#define SPI_IF_RFHF_Pos				2		//д1����
#define SPI_IF_RFHF_Msk				(0x01 << SPI_IF_RFHF_Pos)
#define SPI_IF_TFE_Pos				3		//д1����
#define SPI_IF_TFE_Msk				(0x01 << SPI_IF_TFE_Pos)
#define SPI_IF_TFHF_Pos				4		//д1����
#define SPI_IF_TFHF_Msk				(0x01 << SPI_IF_TFHF_Pos)
#define SPI_IF_RFTHR_Pos			5		//д1����
#define SPI_IF_RFTHR_Msk			(0x01 << SPI_IF_RFTHR_Pos)
#define SPI_IF_TFTHR_Pos			6		//д1����
#define SPI_IF_TFTHR_Msk			(0x01 << SPI_IF_TFTHR_Pos)
#define SPI_IF_WTC_Pos				8		//Word Transmit Complete��ÿ�������һ����������Ӳ����1
#define SPI_IF_WTC_Msk				(0x01 << SPI_IF_WTC_Pos)
#define SPI_IF_FTC_Pos				9		//Frame Transmit Complete��WTC��λʱ��TX FIFO�ǿյģ���FTC��λ
#define SPI_IF_FTC_Msk				(0x01 << SPI_IF_FTC_Pos)
#define SPI_IF_CSFALL_Pos			10
#define SPI_IF_CSFALL_Msk			(0x01 << SPI_IF_CSFALL_Pos)
#define SPI_IF_CSRISE_Pos			11
#define SPI_IF_CSRISE_Msk			(0x01 << SPI_IF_CSRISE_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t IE;
	
	__IO uint32_t IF;
	
	__IO uint32_t SMPNUM;
	
	__IO uint32_t SMPTIM;
	
	__IO uint32_t SEQTRG;
		
	__IO uint32_t SEQ0CHN;
	
	__IO uint32_t SEQ1CHN;
	
	__IO uint32_t SEQ0CHK;
	    
	__IO uint32_t SEQ1CHK;
	
		 uint32_t RESERVED[2];
	
	__IO uint32_t DATA[10];					// ͨ�� 10-15 ת�������ͨ�� SEQ0DMA��SEQ1DMA ��ȡ
	
		 uint32_t RESERVED2[6];
	
	__IO uint32_t SEQ0DMA;
	
	__IO uint32_t SEQ1DMA;
	
		 uint32_t RESERVED3[98];
	
	__IO uint32_t START;
} ADC_TypeDef;


#define ADC_CR_PWDN_Pos				0		//1 Power Down   0 ��������ģʽ��д 0 ����ȴ� 32 ����������
#define ADC_CR_PWDN_Msk				(0x01 << ADC_CR_PWDN_Pos)
#define ADC_CR_RESET_Pos			1		//ģ��IP�ڲ��߼���λ��Ӳ���Զ�����
#define ADC_CR_RESET_Msk			(0x01 << ADC_CR_RESET_Pos)
#define ADC_CR_SEQ0DMAEN_Pos		4
#define ADC_CR_SEQ0DMAEN_Msk		(0x01 << ADC_CR_SEQ0DMAEN_Pos)
#define ADC_CR_SEQ1DMAEN_Pos		5
#define ADC_CR_SEQ1DMAEN_Msk		(0x01 << ADC_CR_SEQ1DMAEN_Pos)
#define ADC_CR_AVG_Pos				6 		//
#define ADC_CR_AVG_Msk				(0x03 << ADC_CR_AVG_Pos)
#define ADC_CR_CLKDIV_Pos			8
#define ADC_CR_CLKDIV_Msk			(0x1F << ADC_CR_CLKDIV_Pos)
#define ADC_CR_BITS_Pos				13 		//ת�����λ����0 12-bit   1 10-bit
#define ADC_CR_BITS_Msk				(0x01 << ADC_CR_BITS_Pos)
#define ADC_CR_CMPPWS_Pos			14		// ADC CMP power-save enable
#define ADC_CR_CMPPWS_Msk			(0x01 << ADC_CR_CMPPWS_Pos)
#define ADC_CR_VCMPWS_Pos			15		// ADC VCM power-save enable
#define ADC_CR_VCMPWS_Msk			(0x01 << ADC_CR_VCMPWS_Pos)

#define ADC_IE_SEQ0EOC_Pos			0
#define ADC_IE_SEQ0EOC_Msk			(0x01 << ADC_IE_SEQ0EOC_Pos)
#define ADC_IE_SEQ0MAX_Pos			1
#define ADC_IE_SEQ0MAX_Msk			(0x01 << ADC_IE_SEQ0MAX_Pos)
#define ADC_IE_SEQ0MIN_Pos			2
#define ADC_IE_SEQ0MIN_Msk			(0x01 << ADC_IE_SEQ0MIN_Pos)
#define ADC_IE_SEQ1EOC_Pos			8
#define ADC_IE_SEQ1EOC_Msk			(0x01 << ADC_IE_SEQ1EOC_Pos)
#define ADC_IE_SEQ1MAX_Pos			9
#define ADC_IE_SEQ1MAX_Msk			(0x01 << ADC_IE_SEQ1MAX_Pos)
#define ADC_IE_SEQ1MIN_Pos			10
#define ADC_IE_SEQ1MIN_Msk			(0x01 << ADC_IE_SEQ1MIN_Pos)

#define ADC_IF_SEQ0EOC_Pos			0
#define ADC_IF_SEQ0EOC_Msk			(0x01 << ADC_IF_SEQ0EOC_Pos)
#define ADC_IF_SEQ0MAX_Pos			1
#define ADC_IF_SEQ0MAX_Msk			(0x01 << ADC_IF_SEQ0MAX_Pos)
#define ADC_IF_SEQ0MIN_Pos			2
#define ADC_IF_SEQ0MIN_Msk			(0x01 << ADC_IF_SEQ0MIN_Pos)
#define ADC_IF_SEQ0BRK_Pos			3		//CPU����������PWM������ϣ�״̬λ���������ж�
#define ADC_IF_SEQ0BRK_Msk			(0x01 << ADC_IF_SEQ0BRK_Pos)
#define ADC_IF_SEQ1EOC_Pos			8
#define ADC_IF_SEQ1EOC_Msk			(0x01 << ADC_IF_SEQ1EOC_Pos)
#define ADC_IF_SEQ1MAX_Pos			9
#define ADC_IF_SEQ1MAX_Msk			(0x01 << ADC_IF_SEQ1MAX_Pos)
#define ADC_IF_SEQ1MIN_Pos			10
#define ADC_IF_SEQ1MIN_Msk			(0x01 << ADC_IF_SEQ1MIN_Pos)
#define ADC_IF_SEQ1BRK_Pos			11
#define ADC_IF_SEQ1BRK_Msk			(0x01 << ADC_IF_SEQ1BRK_Pos)

#define ADC_SMPNUM_SEQ0_Pos			0
#define ADC_SMPNUM_SEQ0_Msk			(0xFF << ADC_SMPNUM_SEQ0_Pos)
#define ADC_SMPNUM_SEQ1_Pos			8
#define ADC_SMPNUM_SEQ1_Msk			(0xFF << ADC_SMPNUM_SEQ1_Pos)

#define ADC_SMPTIM_SEQ0_Pos			0
#define ADC_SMPTIM_SEQ0_Msk			(0xFF << ADC_SMPTIM_SEQ0_Pos)
#define ADC_SMPTIM_SEQ1_Pos			8
#define ADC_SMPTIM_SEQ1_Msk			(0xFF << ADC_SMPTIM_SEQ1_Pos)

#define ADC_SEQTRG_SEQ0_Pos			0
#define ADC_SEQTRG_SEQ0_Msk			(0xFF << ADC_SEQTRG_SEQ0_Pos)
#define ADC_SEQTRG_SEQ1_Pos			8
#define ADC_SEQTRG_SEQ1_Msk			(0xFF << ADC_SEQTRG_SEQ1_Pos)

#define ADC_SEQ0CHN_CH0_Pos			0
#define ADC_SEQ0CHN_CH0_Msk			(0x0F << ADC_SEQ0CHN_CH0_Pos)
#define ADC_SEQ0CHN_CH1_Pos			4
#define ADC_SEQ0CHN_CH1_Msk			(0x0F << ADC_SEQ0CHN_CH1_Pos)
#define ADC_SEQ0CHN_CH2_Pos			8
#define ADC_SEQ0CHN_CH2_Msk			(0x0F << ADC_SEQ0CHN_CH2_Pos)
#define ADC_SEQ0CHN_CH3_Pos			12
#define ADC_SEQ0CHN_CH3_Msk			(0x0F << ADC_SEQ0CHN_CH3_Pos)
#define ADC_SEQ0CHN_CH4_Pos			16
#define ADC_SEQ0CHN_CH4_Msk			(0x0F << ADC_SEQ0CHN_CH4_Pos)
#define ADC_SEQ0CHN_CH5_Pos			20
#define ADC_SEQ0CHN_CH5_Msk			(0x0F << ADC_SEQ0CHN_CH5_Pos)
#define ADC_SEQ0CHN_CH6_Pos			24
#define ADC_SEQ0CHN_CH6_Msk			(0x0F << ADC_SEQ0CHN_CH6_Pos)
#define ADC_SEQ0CHN_CH7_Pos			28
#define ADC_SEQ0CHN_CH7_Msk			(0x0F << ADC_SEQ0CHN_CH7_Pos)

#define ADC_SEQ1CHN_CH0_Pos			0
#define ADC_SEQ1CHN_CH0_Msk			(0x0F << ADC_SEQ1CHN_CH0_Pos)
#define ADC_SEQ1CHN_CH1_Pos			4
#define ADC_SEQ1CHN_CH1_Msk			(0x0F << ADC_SEQ1CHN_CH1_Pos)
#define ADC_SEQ1CHN_CH2_Pos			8
#define ADC_SEQ1CHN_CH2_Msk			(0x0F << ADC_SEQ1CHN_CH2_Pos)
#define ADC_SEQ1CHN_CH3_Pos			12
#define ADC_SEQ1CHN_CH3_Msk			(0x0F << ADC_SEQ1CHN_CH3_Pos)
#define ADC_SEQ1CHN_CH4_Pos			16
#define ADC_SEQ1CHN_CH4_Msk			(0x0F << ADC_SEQ1CHN_CH4_Pos)
#define ADC_SEQ1CHN_CH5_Pos			20
#define ADC_SEQ1CHN_CH5_Msk			(0x0F << ADC_SEQ1CHN_CH5_Pos)
#define ADC_SEQ1CHN_CH6_Pos			24
#define ADC_SEQ1CHN_CH6_Msk			(0x0F << ADC_SEQ1CHN_CH6_Pos)
#define ADC_SEQ1CHN_CH7_Pos			28
#define ADC_SEQ1CHN_CH7_Msk			(0x0F << ADC_SEQ1CHN_CH7_Pos)

#define ADC_SEQ0CHK_MAX_Pos			0
#define ADC_SEQ0CHK_MAX_Msk			(0xFFF<< ADC_SEQ0CHK_MAX_Pos)
#define ADC_SEQ0CHK_MIN_Pos			16
#define ADC_SEQ0CHK_MIN_Msk			(0xFFF<< ADC_SEQ0CHK_MIN_Pos)

#define ADC_SEQ1CHK_MAX_Pos			0
#define ADC_SEQ1CHK_MAX_Msk			(0xFFF<< ADC_SEQ1CHK_MAX_Pos)
#define ADC_SEQ1CHK_MIN_Pos			16
#define ADC_SEQ1CHK_MIN_Msk			(0xFFF<< ADC_SEQ1CHK_MIN_Pos)

#define ADC_DATA_DATA_Pos			0
#define ADC_DATA_DATA_Msk			(0xFFF<< ADC_DATA_DATA_Pos)
#define ADC_DATA_FLAG_Pos			16		//0 ���ϴζ�ȡ��������   1 ��������   2 ���������ݸ���
#define ADC_DATA_FLAG_Msk			(0x03 << ADC_DATA_FLAG_Pos)

#define ADC_SEQ0DMA_DATA_Pos		0
#define ADC_SEQ0DMA_DATA_Msk		(0xFFF<< ADC_SEQ0DMA_DATA_Pos)
#define ADC_SEQ0DMA_CHNUM_Pos		12
#define ADC_SEQ0DMA_CHNUM_Msk		(0x0F << ADC_SEQ0DMA_CHNUM_Pos)
#define ADC_SEQ0DMA_FLAG_Pos		16
#define ADC_SEQ0DMA_FLAG_Msk		(0x03 << ADC_SEQ0DMA_FLAG_Pos)

#define ADC_SEQ1DMA_DATA_Pos		0
#define ADC_SEQ1DMA_DATA_Msk		(0xFFF<< ADC_SEQ1DMA_DATA_Pos)
#define ADC_SEQ1DMA_CHNUM_Pos		12
#define ADC_SEQ1DMA_CHNUM_Msk		(0x0F << ADC_SEQ1DMA_CHNUM_Pos)
#define ADC_SEQ1DMA_FLAG_Pos		16
#define ADC_SEQ1DMA_FLAG_Msk		(0x03 << ADC_SEQ1DMA_FLAG_Pos)

#define ADC_START_ADC0SEQ0_Pos		0
#define ADC_START_ADC0SEQ0_Msk		(0x01 << ADC_START_ADC0SEQ0_Pos)
#define ADC_START_ADC0SEQ1_Pos		1
#define ADC_START_ADC0SEQ1_Msk		(0x01 << ADC_START_ADC0SEQ1_Pos)
#define ADC_START_ADC0BUSY_Pos		2
#define ADC_START_ADC0BUSY_Msk		(0x01 << ADC_START_ADC0BUSY_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t OCR;
	
	__IO uint32_t BRKCR;
	
	__IO uint32_t BRKIN;
	
		 uint32_t RESERVED[4];
	
	__IO uint32_t PERIOD;                   //[15:0] ����
	
	__IO uint32_t CMPA;                   	//[15:0] A·��ת��Ƚ�ֵ
	
	__IO uint32_t CMPB;						//[15:0] B·��ת��Ƚ�ֵ
	
	__IO uint32_t DZA;                      //[9:0] ����
	
	__IO uint32_t DZB;
	
	__IO uint32_t CMPA2;					//�ǶԳ����Ķ���ģʽ�£����¼��������У�A·��ת��Ƚ�ֵ
	
	__IO uint32_t CMPB2;					//�ǶԳ����Ķ���ģʽ�£����¼��������У�B·��ת��Ƚ�ֵ
	
		 uint32_t RESERVED2[5];
	
	__IO uint32_t OVFTRG;
	
	__IO uint32_t CMPTRG;
	
	__IO uint32_t CMPTRG2;
	
		 uint32_t RESERVED3;
	
	__IO uint32_t EVMUX;
	
    __IO uint32_t EVMSK;
	
		 uint32_t RESERVED4[2];
	
	__IO uint32_t IE;
	
	__IO uint32_t IF;
	
	__IO uint32_t VALUE;
	
	__IO uint32_t SR;
} PWM_TypeDef;


#define PWM_CR_MODE_Pos				0		//0 ���ض���ģʽ   1 ���Ķ���ģʽ   2 �ǶԳ����Ķ���ģʽ
#define PWM_CR_MODE_Msk				(0x03 << PWM_CR_MODE_Pos)
#define PWM_CR_MULT_Pos				2		//0 ���μ���ģʽ   1 ��μ���ģʽ
#define PWM_CR_MULT_Msk				(0x01 << PWM_CR_MULT_Pos)
#define PWM_CR_DIR_Pos				3		//�������������� 0 ���ϼ���   1 ���¼���
#define PWM_CR_DIR_Msk				(0x01 << PWM_CR_DIR_Pos)
#define PWM_CR_CLKSRC_Pos			4		//����ʱ��Դ��0 ϵͳʱ��   1 PWM_PULSE0����   2 PWM_PULSE1����
#define PWM_CR_CLKSRC_Msk			(0x03 << PWM_CR_CLKSRC_Pos)
#define PWM_CR_CLKDIV_Pos			6		//����ʱ��Ԥ��Ƶ�� 0 1��Ƶ   1 2��Ƶ   ...   1023 1024��Ƶ
#define PWM_CR_CLKDIV_Msk			(0x3FF<< PWM_CR_CLKDIV_Pos)
#define PWM_CR_RPTNUM_Pos			16		//������������ٴ�ִ��һ�μĴ������أ�0 1��   1 2��   ...   255 256��
#define PWM_CR_RPTNUM_Msk			(0xFF << PWM_CR_RPTNUM_Pos)

#define PWM_OCR_IDLEA_Pos			0		//A·����ʱ�����ƽ
#define PWM_OCR_IDLEA_Msk			(0x01 << PWM_OCR_IDLEA_Pos)
#define PWM_OCR_IDLEB_Pos			1		//B·����ʱ�����ƽ
#define PWM_OCR_IDLEB_Msk			(0x01 << PWM_OCR_IDLEB_Pos)
#define PWM_OCR_IDLEAN_Pos			2		//AN·����ʱ�����ƽ
#define PWM_OCR_IDLEAN_Msk			(0x01 << PWM_OCR_IDLEAN_Pos)
#define PWM_OCR_IDLEBN_Pos			3		//BN·����ʱ�����ƽ
#define PWM_OCR_IDLEBN_Msk			(0x01 << PWM_OCR_IDLEBN_Pos)
#define PWM_OCR_INVA_Pos			4		//A·����Ƿ�ȡ��
#define PWM_OCR_INVA_Msk			(0x01 << PWM_OCR_INVA_Pos)
#define PWM_OCR_INVB_Pos			5		//B·����Ƿ�ȡ��
#define PWM_OCR_INVB_Msk			(0x01 << PWM_OCR_INVB_Pos)
#define PWM_OCR_INVAN_Pos			6		//AN·����Ƿ�ȡ��
#define PWM_OCR_INVAN_Msk			(0x01 << PWM_OCR_INVAN_Pos)
#define PWM_OCR_INVBN_Pos			7		//BN·����Ƿ�ȡ��
#define PWM_OCR_INVBN_Msk			(0x01 << PWM_OCR_INVBN_Pos)
#define PWM_OCR_FORCEA_Pos			8		//A·ǿ�����ʹ�ܣ�ǿ�Ƶ�ƽ�� IDLEA �趨
#define PWM_OCR_FORCEA_Msk			(0x01 << PWM_OCR_FORCEA_Pos)
#define PWM_OCR_FORCEB_Pos			9
#define PWM_OCR_FORCEB_Msk			(0x01 << PWM_OCR_FORCEB_Pos)
#define PWM_OCR_FORCEAN_Pos			10
#define PWM_OCR_FORCEAN_Msk			(0x01 << PWM_OCR_FORCEAN_Pos)
#define PWM_OCR_FORCEBN_Pos			11
#define PWM_OCR_FORCEBN_Msk			(0x01 << PWM_OCR_FORCEBN_Pos)

#define PWM_BRKCR_OUTA_Pos			0		//ɲ��״̬��A·�����ƽ
#define PWM_BRKCR_OUTA_Msk			(0x01 << PWM_BRKCR_OUTA_Pos)
#define PWM_BRKCR_OFFA_Pos			1		//ɲ���źų���ʱA·�����0 �����ָ��������   1 ���ֵ�ǰ���ֱ������������ٻָ��������
#define PWM_BRKCR_OFFA_Msk			(0x01 << PWM_BRKCR_OFFA_Pos)
#define PWM_BRKCR_OUTB_Pos			4		//ɲ��״̬��B·�����ƽ
#define PWM_BRKCR_OUTB_Msk			(0x01 << PWM_BRKCR_OUTB_Pos)
#define PWM_BRKCR_OFFB_Pos			5		//ɲ���źų���ʱB·�����0 �����ָ��������   1 ���ֵ�ǰ���ֱ������������ٻָ��������
#define PWM_BRKCR_OFFB_Msk			(0x01 << PWM_BRKCR_OFFB_Pos)
#define PWM_BRKCR_OUTAN_Pos			8		//ɲ��״̬��AN·�����ƽ
#define PWM_BRKCR_OUTAN_Msk			(0x01 << PWM_BRKCR_OUTAN_Pos)
#define PWM_BRKCR_OUTBN_Pos			9		//ɲ��״̬��BN·�����ƽ
#define PWM_BRKCR_OUTBN_Msk			(0x01 << PWM_BRKCR_OUTBN_Pos)
#define PWM_BRKCR_STPCNT_Pos		10		//ɲ��״̬���Ƿ�ֹͣ��������1 ֹͣ������   0 ��������
#define PWM_BRKCR_STPCNT_Msk		(0x01 << PWM_BRKCR_STPCNT_Pos)
#define PWM_BRKCR_SWHALT_Pos		16		//��ǰ�Ƿ������ɲ��״̬
#define PWM_BRKCR_SWHALT_Msk		(0x01 << PWM_BRKCR_SWHALT_Pos)
#define PWM_BRKCR_HWHALT_Pos		17		//��ǰ�Ƿ���Ӳ��ɲ��״̬
#define PWM_BRKCR_HWHALT_Msk		(0x01 << PWM_BRKCR_HWHALT_Pos)

#define PWM_BRKIN_BRK0A_Pos			0		//A·�Ƿ���ɲ������PWM_BRK0Ӱ��
#define PWM_BRKIN_BRK0A_Msk			(0x01 << PWM_BRKIN_BRK0A_Pos)
#define PWM_BRKIN_BRK1A_Pos			1
#define PWM_BRKIN_BRK1A_Msk			(0x01 << PWM_BRKIN_BRK1A_Pos)
#define PWM_BRKIN_BRK2A_Pos			2
#define PWM_BRKIN_BRK2A_Msk			(0x01 << PWM_BRKIN_BRK2A_Pos)
#define PWM_BRKIN_BRK0B_Pos			4
#define PWM_BRKIN_BRK0B_Msk			(0x01 << PWM_BRKIN_BRK0B_Pos)
#define PWM_BRKIN_BRK1B_Pos			5
#define PWM_BRKIN_BRK1B_Msk			(0x01 << PWM_BRKIN_BRK1B_Pos)
#define PWM_BRKIN_BRK2B_Pos			6
#define PWM_BRKIN_BRK2B_Msk			(0x01 << PWM_BRKIN_BRK2B_Pos)

#define PWM_OVFTRG_UPEN_Pos			0		//�������������Triggerʹ��
#define PWM_OVFTRG_UPEN_Msk			(0x01 << PWM_OVFTRG_UPEN_Pos)
#define PWM_OVFTRG_DNEN_Pos			1		//�������������Triggerʹ��
#define PWM_OVFTRG_DNEN_Msk			(0x01 << PWM_OVFTRG_DNEN_Pos)
#define PWM_OVFTRG_MUX_Pos			2		//Trigger�������һ·��0 trig[0]   1 trig[1]   2 trig[2]   ...   7 trig[7]
#define PWM_OVFTRG_MUX_Msk			(0x07 << PWM_OVFTRG_MUX_Pos)

#define PWM_CMPTRG_CMP_Pos			0		//������ֵ��˱Ƚ�ֵ���ʱ����Trigger�ź�
#define PWM_CMPTRG_CMP_Msk			(0xFFFF<<PWM_CMPTRG_CMP_Pos)
#define PWM_CMPTRG_EN_Pos			16
#define PWM_CMPTRG_EN_Msk			(0x01 << PWM_CMPTRG_EN_Pos)
#define PWM_CMPTRG_MUX_Pos			17		//Trigger�������һ·��0 trig[0]   1 trig[1]   2 trig[2]   ...   7 trig[7]
#define PWM_CMPTRG_MUX_Msk			(0x07 << PWM_CMPTRG_MUX_Pos)
#define PWM_CMPTRG_WIDTH_Pos		20		//Trigger����źſ�ȣ�0 �����   1 4������ʱ��   2 8������ʱ��   ...   63 252������ʱ��
#define PWM_CMPTRG_WIDTH_Msk		(0x3F << PWM_CMPTRG_WIDTH_Pos)
#define PWM_CMPTRG_DIR_Pos			28		//0 ���ϼ��������в���Trigger   1 ���¼��������в���Trigger
#define PWM_CMPTRG_DIR_Msk			(0x01 << PWM_CMPTRG_DIR_Pos)
#define PWM_CMPTRG_ATP_Pos			29		//AD�����ź������ڿ��е�λ�ã�0 0/8��   1 1/8��   ...   7 7/8��
#define PWM_CMPTRG_ATP_Msk			(0x07u<< PWM_CMPTRG_ATP_Pos)

#define PWM_CMPTRG2_INTV_Pos		0		//Compare Trigger Interval��0 ÿ���ڴ���   1 ���1���ڴ���һ��   2 ���2���ڴ���һ�� ...
#define PWM_CMPTRG2_INTV_Msk		(0x07 << PWM_CMPTRG2_INTV_Pos)

#define PWM_EVMUX_START_Pos			0
#define PWM_EVMUX_START_Msk			(0x07 << PWM_EVMUX_START_Pos)
#define PWM_EVMUX_STOP_Pos			4
#define PWM_EVMUX_STOP_Msk			(0x07 << PWM_EVMUX_STOP_Pos)
#define PWM_EVMUX_PAUSE_Pos			8
#define PWM_EVMUX_PAUSE_Msk			(0x07 << PWM_EVMUX_PAUSE_Pos)
#define PWM_EVMUX_RELOAD_Pos		12
#define PWM_EVMUX_RELOAD_Msk		(0x07 << PWM_EVMUX_RELOAD_Pos)
#define PWM_EVMUX_MASKA_Pos			16
#define PWM_EVMUX_MASKA_Msk			(0x07 << PWM_EVMUX_MASKA_Pos)
#define PWM_EVMUX_MASKB_Pos			20
#define PWM_EVMUX_MASKB_Msk			(0x07 << PWM_EVMUX_MASKB_Pos)
#define PWM_EVMUX_MASKAN_Pos		24
#define PWM_EVMUX_MASKAN_Msk		(0x07 << PWM_EVMUX_MASKAN_Pos)
#define PWM_EVMUX_MASKBN_Pos		28
#define PWM_EVMUX_MASKBN_Msk		(0x07 << PWM_EVMUX_MASKBN_Pos)

#define PWM_EVMSK_OUTA_Pos			0
#define PWM_EVMSK_OUTA_Msk			(0x01 << PWM_EVMSK_OUTA_Pos)
#define PWM_EVMSK_OUTB_Pos			1
#define PWM_EVMSK_OUTB_Msk			(0x01 << PWM_EVMSK_OUTB_Pos)
#define PWM_EVMSK_OUTAN_Pos			2
#define PWM_EVMSK_OUTAN_Msk			(0x01 << PWM_EVMSK_OUTAN_Pos)
#define PWM_EVMSK_OUTBN_Pos			3
#define PWM_EVMSK_OUTBN_Msk			(0x01 << PWM_EVMSK_OUTBN_Pos)
#define PWM_EVMSK_IMME_Pos			4		//1 MASK������Ч   0 ���������ʱ��Ч
#define PWM_EVMSK_IMME_Msk			(0x01 << PWM_EVMSK_IMME_Pos)
#define PWM_EVMSK_STPCLR_Pos		8		//�ⲿ�¼����¼�����ֹͣʱ�������Ƿ����㣬1 ����   0 ���ֵ�ǰֵ
#define PWM_EVMSK_STPCLR_Msk		(0x01 << PWM_EVMSK_STPCLR_Pos)

#define PWM_IE_UPOVF_Pos			0		//���ϼ���ʱ����������ж�ʹ��
#define PWM_IE_UPOVF_Msk			(0x01 << PWM_IE_UPOVF_Pos)
#define PWM_IE_DNOVF_Pos			1		//���¼���ʱ����������ж�ʹ��
#define PWM_IE_DNOVF_Msk			(0x01 << PWM_IE_DNOVF_Pos)
#define PWM_IE_UPCMPA_Pos			2		//���ϼ���ʱ������ֵ��CMPA����ж�ʹ��
#define PWM_IE_UPCMPA_Msk			(0x01 << PWM_IE_UPCMPA_Pos)
#define PWM_IE_UPCMPB_Pos			3		//���ϼ���ʱ������ֵ��CMPB����ж�ʹ��
#define PWM_IE_UPCMPB_Msk			(0x01 << PWM_IE_UPCMPB_Pos)
#define PWM_IE_DNCMPA_Pos			4		//���¼���ʱ������ֵ��CMPA����ж�ʹ��
#define PWM_IE_DNCMPA_Msk			(0x01 << PWM_IE_DNCMPA_Pos)
#define PWM_IE_DNCMPB_Pos			5		//���¼���ʱ������ֵ��CMPB����ж�ʹ��
#define PWM_IE_DNCMPB_Msk			(0x01 << PWM_IE_DNCMPB_Pos)

#define PWM_IF_UPOVF_Pos			0
#define PWM_IF_UPOVF_Msk			(0x01 << PWM_IF_UPOVF_Pos)
#define PWM_IF_DNOVF_Pos			1
#define PWM_IF_DNOVF_Msk			(0x01 << PWM_IF_DNOVF_Pos)
#define PWM_IF_UPCMPA_Pos			2
#define PWM_IF_UPCMPA_Msk			(0x01 << PWM_IF_UPCMPA_Pos)
#define PWM_IF_UPCMPB_Pos			3
#define PWM_IF_UPCMPB_Msk			(0x01 << PWM_IF_UPCMPB_Pos)
#define PWM_IF_DNCMPA_Pos			4
#define PWM_IF_DNCMPA_Msk			(0x01 << PWM_IF_DNCMPA_Pos)
#define PWM_IF_DNCMPB_Pos			5
#define PWM_IF_DNCMPB_Msk			(0x01 << PWM_IF_DNCMPB_Pos)

#define PWM_SR_STAT_Pos				0		//0 IDLE   1 ACTIVE   2 PAUSE
#define PWM_SR_STAT_Msk				(0x03 << PWM_SR_STAT_Pos)
#define PWM_SR_DIR_Pos				4		//0 ���ϼ���   1 ���¼���
#define PWM_SR_DIR_Msk				(0x01 << PWM_SR_DIR_Pos)
#define PWM_SR_OUTA_Pos				5
#define PWM_SR_OUTA_Msk				(0x01 << PWM_SR_OUTA_Pos)
#define PWM_SR_OUTB_Pos				6
#define PWM_SR_OUTB_Msk				(0x01 << PWM_SR_OUTB_Pos)
#define PWM_SR_OUTAN_Pos			7
#define PWM_SR_OUTAN_Msk			(0x01 << PWM_SR_OUTAN_Pos)
#define PWM_SR_OUTBN_Pos			8
#define PWM_SR_OUTBN_Msk			(0x01 << PWM_SR_OUTBN_Pos)


typedef struct {
	__IO uint32_t START;
	
	__IO uint32_t SWBRK;					//Software Brake�����ɲ��
    
    __IO uint32_t RESET;
	
	union {
		__IO uint32_t RELOADEN;
		
		__IO uint32_t RESTART;
	};
	
    __IO uint32_t PULSE;
	
    __IO uint32_t FILTER;					//�ⲿ�ź��˲���0 ���˲�   1 4��PCLK����   2 8��PCLK����   3 16��PCLK����
	
    __IO uint32_t BRKPOL;					//ɲ���źż��ԣ�
	
    __IO uint32_t BRKIE;
    
	union {
		__IO uint32_t BRKIF;
		
		__IO uint32_t BRKSR;
	};
	
	__IO uint32_t EVSR;
	
	__IO uint32_t SWEV;
} PWMG_TypeDef;


#define PWMG_START_PWM0_Pos			0
#define PWMG_START_PWM0_Msk			(0x01 << PWMG_START_PWM0_Pos)
#define PWMG_START_PWM1_Pos			1
#define PWMG_START_PWM1_Msk			(0x01 << PWMG_START_PWM1_Pos)

#define PWMG_SWBRK_PWM0A_Pos		0
#define PWMG_SWBRK_PWM0A_Msk		(0x01 << PWMG_SWBRK_PWM0A_Pos)
#define PWMG_SWBRK_PWM1A_Pos		1
#define PWMG_SWBRK_PWM1A_Msk		(0x01 << PWMG_SWBRK_PWM1A_Pos)
#define PWMG_SWBRK_PWM0B_Pos		8
#define PWMG_SWBRK_PWM0B_Msk		(0x01 << PWMG_SWBRK_PWM0B_Pos)
#define PWMG_SWBRK_PWM1B_Pos		9
#define PWMG_SWBRK_PWM1B_Msk		(0x01 << PWMG_SWBRK_PWM1B_Pos)

#define PWMG_RESET_PWM0_Pos			0
#define PWMG_RESET_PWM0_Msk			(0x01 << PWMG_RESET_PWM0_Pos)
#define PWMG_RESET_PWM1_Pos			1
#define PWMG_RESET_PWM1_Msk			(0x01 << PWMG_RESET_PWM1_Pos)

#define PWMG_RELOADEN_PWM0_Pos		0
#define PWMG_RELOADEN_PWM0_Msk		(0x01 << PWMG_RELOADEN_PWM0_Pos)
#define PWMG_RELOADEN_PWM1_Pos		1
#define PWMG_RELOADEN_PWM1_Msk		(0x01 << PWMG_RELOADEN_PWM1_Pos)

#define PWMG_RESTART_PWM0_Pos		8
#define PWMG_RESTART_PWM0_Msk		(0x01 << PWMG_RESTART_PWM0_Pos)
#define PWMG_RESTART_PWM1_Pos		9
#define PWMG_RESTART_PWM1_Msk		(0x01 << PWMG_RESTART_PWM1_Pos)

#define PWMG_PULSE_EDGE0_Pos		0		//PWM_PULSE0 �������أ�0 ������   1 �½���
#define PWMG_PULSE_EDGE0_Msk		(0x01 << PWMG_PULSE_EDGE0_Pos)
#define PWMG_PULSE_EDGE1_Pos		1
#define PWMG_PULSE_EDGE1_Msk		(0x01 << PWMG_PULSE_EDGE1_Pos)

#define PWMG_BRKPOL_BRK0_Pos		0		//PWMG_BRK0 ɲ���źż��ԣ�0 �͵�ƽɲ��   1 �ߵ�ƽɲ��
#define PWMG_BRKPOL_BRK0_Msk		(0x01 << PWMG_BRKPOL_BRK0_Pos)
#define PWMG_BRKPOL_BRK1_Pos		1
#define PWMG_BRKPOL_BRK1_Msk		(0x01 << PWMG_BRKPOL_BRK1_Pos)
#define PWMG_BRKPOL_BRK2_Pos		2
#define PWMG_BRKPOL_BRK2_Msk		(0x01 << PWMG_BRKPOL_BRK2_Pos)

#define PWMG_BRKIE_BRK0_Pos			0
#define PWMG_BRKIE_BRK0_Msk			(0x01 << PWMG_BRKIE_BRK0_Pos)
#define PWMG_BRKIE_BRK1_Pos			1
#define PWMG_BRKIE_BRK1_Msk			(0x01 << PWMG_BRKIE_BRK1_Pos)
#define PWMG_BRKIE_BRK2_Pos			2
#define PWMG_BRKIE_BRK2_Msk			(0x01 << PWMG_BRKIE_BRK2_Pos)

#define PWMG_BRKIF_BRK0_Pos			0
#define PWMG_BRKIF_BRK0_Msk			(0x01 << PWMG_BRKIF_BRK0_Pos)
#define PWMG_BRKIF_BRK1_Pos			1
#define PWMG_BRKIF_BRK1_Msk			(0x01 << PWMG_BRKIF_BRK1_Pos)
#define PWMG_BRKIF_BRK2_Pos			2
#define PWMG_BRKIF_BRK2_Msk			(0x01 << PWMG_BRKIF_BRK2_Pos)

#define PWMG_BRKSR_BRK0_Pos			4		//ɲ�����ŵ�ƽֵ
#define PWMG_BRKSR_BRK0_Msk			(0x01 << PWMG_BRKSR_BRK0_Pos)
#define PWMG_BRKSR_BRK1_Pos			5
#define PWMG_BRKSR_BRK1_Msk			(0x01 << PWMG_BRKSR_BRK1_Pos)
#define PWMG_BRKSR_BRK2_Pos			6
#define PWMG_BRKSR_BRK2_Msk			(0x01 << PWMG_BRKSR_BRK2_Pos)

#define PWMG_EVSR_EV0_Pos			0		//�ⲿ�¼��źŵ�ƽֵ
#define PWMG_EVSR_EV0_Msk			(0x01 << PWMG_EVSR_EV0_Pos)
#define PWMG_EVSR_EV1_Pos			1
#define PWMG_EVSR_EV1_Msk			(0x01 << PWMG_EVSR_EV1_Pos)
#define PWMG_EVSR_EV2_Pos			2
#define PWMG_EVSR_EV2_Msk			(0x01 << PWMG_EVSR_EV2_Pos)
#define PWMG_EVSR_EV3_Pos			3
#define PWMG_EVSR_EV3_Msk			(0x01 << PWMG_EVSR_EV3_Pos)
#define PWMG_EVSR_EV4_Pos			4
#define PWMG_EVSR_EV4_Msk			(0x01 << PWMG_EVSR_EV4_Pos)
#define PWMG_EVSR_EV5_Pos			5
#define PWMG_EVSR_EV5_Msk			(0x01 << PWMG_EVSR_EV5_Pos)
#define PWMG_EVSR_EV6_Pos			6
#define PWMG_EVSR_EV6_Msk			(0x01 << PWMG_EVSR_EV6_Pos)

#define PWMG_SWEV_EV2_Pos			0
#define PWMG_SWEV_EV2_Msk			(0x01 << PWMG_SWEV_EV2_Pos)
#define PWMG_SWEV_EV3_Pos			1
#define PWMG_SWEV_EV3_Msk			(0x01 << PWMG_SWEV_EV3_Pos)
#define PWMG_SWEV_EV4_Pos			2
#define PWMG_SWEV_EV4_Msk			(0x01 << PWMG_SWEV_EV4_Pos)




typedef struct {    
	__IO uint32_t IF;						//Interrupt Flag
    
	__IO uint32_t IFC;						//Interrupt Flag Clear
    	
		 uint32_t RESERVED[2];
		
	struct {
		__IO uint32_t MUX;
		
		__IO uint32_t CR;
		
		__IO uint32_t NDT;					//Number of data to transfer
		
		__IO uint32_t PAR;					//Peripheral address register
		
		__IO uint32_t MAR;					//Memory address register
		
			 uint32_t RESERVED[3];
	} CH[1];
} DMA_TypeDef;


#define DMA_IF_GLB0_Pos				0		//Channel 0 global interrupt
#define DMA_IF_GLB0_Msk				(0x01 << DMA_IF_GLB0_Pos)
#define DMA_IF_DONE0_Pos			1		//Channel 0 transfer done
#define DMA_IF_DONE0_Msk			(0x01 << DMA_IF_DONE0_Pos)
#define DMA_IF_HALF0_Pos			2		//Channel 0 half transfer
#define DMA_IF_HALF0_Msk			(0x01 << DMA_IF_HALF0_Pos)
#define DMA_IF_ERR0_Pos				3		//Channel 0 transfer error
#define DMA_IF_ERR0_Msk				(0x01 << DMA_IF_ERR0_Pos)

#define DMA_IFC_GLB0_Pos			0
#define DMA_IFC_GLB0_Msk			(0x01 << DMA_IFC_GLB0_Pos)
#define DMA_IFC_DONE0_Pos			1
#define DMA_IFC_DONE0_Msk			(0x01 << DMA_IFC_DONE0_Pos)
#define DMA_IFC_HALF0_Pos			2
#define DMA_IFC_HALF0_Msk			(0x01 << DMA_IFC_HALF0_Pos)
#define DMA_IFC_ERR0_Pos			3
#define DMA_IFC_ERR0_Msk			(0x01 << DMA_IFC_ERR0_Pos)

#define DMA_MUX_MRDHSSIG_Pos		0		//memory read  handshake signal
#define DMA_MUX_MRDHSSIG_Msk		(0x03 << DMA_MUX_MRDHSSIG_Pos)
#define DMA_MUX_MRDHSEN_Pos			3		//memory read  handshake enable
#define DMA_MUX_MRDHSEN_Msk			(0x01 << DMA_MUX_MRDHSEN_Pos)
#define DMA_MUX_MWRHSSIG_Pos		4		//memory write handshake signal
#define DMA_MUX_MWRHSSIG_Msk		(0x03 << DMA_MUX_MWRHSSIG_Pos)
#define DMA_MUX_MWRHSEN_Pos			7		//memory write handshake enable
#define DMA_MUX_MWRHSEN_Msk			(0x01 << DMA_MUX_MWRHSEN_Pos)
#define DMA_MUX_EXTHSSIG_Pos		8		//�ⲿ�����źţ�0 TIMR0   1 TIMR1   2 TIMR2   3 TIMR3   4 TIMR4   5 DMA_TRIG0   6 DMA_TRIG1
#define DMA_MUX_EXTHSSIG_Msk		(0x07 << DMA_MUX_EXTHSSIG_Pos)
#define DMA_MUX_EXTHSEN_Pos			11
#define DMA_MUX_EXTHSEN_Msk			(0x01 << DMA_MUX_EXTHSEN_Pos)

#define DMA_CR_EN_Pos				0
#define DMA_CR_EN_Msk				(0x01 << DMA_CR_EN_Pos)
#define DMA_CR_DONEIE_Pos			1
#define DMA_CR_DONEIE_Msk			(0x01 << DMA_CR_DONEIE_Pos)
#define DMA_CR_HALFIE_Pos			2
#define DMA_CR_HALFIE_Msk			(0x01 << DMA_CR_HALFIE_Pos)
#define DMA_CR_ERRIE_Pos			3
#define DMA_CR_ERRIE_Msk			(0x01 << DMA_CR_ERRIE_Pos)
#define DMA_CR_DIR_Pos				4
#define DMA_CR_DIR_Msk				(0x01 << DMA_CR_DIR_Pos)
#define DMA_CR_CIRC_Pos				5
#define DMA_CR_CIRC_Msk				(0x01 << DMA_CR_CIRC_Pos)
#define DMA_CR_PINC_Pos				6
#define DMA_CR_PINC_Msk				(0x01 << DMA_CR_PINC_Pos)
#define DMA_CR_MINC_Pos				7
#define DMA_CR_MINC_Msk				(0x01 << DMA_CR_MINC_Pos)
#define DMA_CR_PSIZ_Pos				8
#define DMA_CR_PSIZ_Msk				(0x03 << DMA_CR_PSIZ_Pos)
#define DMA_CR_MSIZ_Pos				10
#define DMA_CR_MSIZ_Msk				(0x03 << DMA_CR_MSIZ_Pos)
#define DMA_CR_PL_Pos				12
#define DMA_CR_PL_Msk				(0x0F << DMA_CR_PL_Pos)
#define DMA_CR_MEM2MEM_Pos			16
#define DMA_CR_MEM2MEM_Msk			(0x01 << DMA_CR_MEM2MEM_Pos)

#define DMA_NDT_LEN_Pos				0		//ͨ���ر�ʱ��д��Ҫ��������ݸ�����ͨ��ʹ�ܺ�ָʾʣ��Ĵ�����������Ŀ
#define DMA_NDT_LEN_Msk				(0xFFFFFF << DMA_NDT_LEN_Pos)
#define DMA_NDT_HALF_Pos			24		// After the specified number of data is transmitted, set the DMA->IF.HALF interrupt flag bit
											// 1 1/8 * LEN, 2 2/8 * LEN, ...
#define DMA_NDT_HALF_Msk			(0x07 << DMA_NDT_HALF_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t SR;
	
	__IO uint32_t IE;						//[0] ��������ж�ʹ��
	
	     uint32_t RESERVED;
	
	__IO uint32_t DIVIDEND;					//������
	
	__IO uint32_t DIVISOR;					//����
	
	__IO uint32_t QUO;						//��
	
	__IO uint32_t REMAIN;					//����
} DIV_TypeDef;


#define DIV_CR_DIVGO_Pos			0		//д1�����������㣬������ɺ��Զ�����
#define DIV_CR_DIVGO_Msk			(0x01 << DIV_CR_DIVGO_Pos)
#define DIV_CR_DIVSIGN_Pos			1		//0 �з��ų���    1 �޷��ų���
#define DIV_CR_DIVSIGN_Msk			(0x01 << DIV_CR_DIVSIGN_Pos)

#define DIV_SR_DIVEND_Pos			0		//����������ɱ�־��д1����
#define DIV_SR_DIVEND_Msk			(0x01 << DIV_SR_DIVEND_Pos)
#define DIV_SR_DIVBUSY_Pos			1		//1 �������������
#define DIV_SR_DIVBUSY_Msk			(0x01 << DIV_SR_DIVBUSY_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t STA;						// start
	
	__IO uint32_t IE;
	
	__IO uint32_t SR;
	
		 uint32_t RSVD;
	
	__IO uint32_t CLARKIAB;					// CLARK input a and b
	
		 uint32_t RSVD2;
	
	__IO uint32_t CLARKIC;					// CLARK input c
	
	__I  uint32_t CLARKOAB;					// CLARK output ��and ��
	
		 uint32_t RSVD3;
	
	__IO uint32_t PARKIAB;					// PARK input ��and ��
	
		 uint32_t RSVD4;
	
	__IO uint32_t PARKRAD;					// PARK sin and cos value of ��
	
		 uint32_t RSVD5;
	
	__I  uint32_t PARKODQ;					// PARK output d and q
	
		 uint32_t RSVD6;
		 
	__IO uint32_t PIDIDQ;					// PID input d and q
	
		 uint32_t RSVD7;
	
	__IO uint32_t PIDREF;					// PID reference value of d and q
	
		 uint32_t RSVD8;
	
	__IO uint32_t PIDMAX;					// PID max value of d and q
	
		 uint32_t RSVD9;
	
	__IO uint32_t PIDMIN;					// PID min value of d and q
	
		 uint32_t RSVD10;
	
	__IO uint32_t PIDKP;					// PID Kp parameter for d and q
	
		 uint32_t RSVD11;
	
	__IO uint32_t PIDKI;					// PID Ki parameter for d and q
	
		 uint32_t RSVD12;
	
	__IO uint32_t PIDLERR;					// PID last error for d and q
	
		 uint32_t RSVD13;
	
	__IO uint32_t PIDLOUT;					// PID last output for d and q
	
		 uint32_t RSVD14;
	
	__I  uint32_t PIDODQ;					// PID output d and q
	
		 uint32_t RSVD15;
	
	__IO uint32_t IPARKIDQ;					// iPARK input d and q
	
		 uint32_t RSVD16;
	
	__IO uint32_t IPARKRAD;					// iPARK sin and cos value of ��
	
		 uint32_t RSVD17;
	
	__I  uint32_t IPARKOAB;					// iPARK output ��and ��
	
		 uint32_t RSVD18[2];
	
	__IO uint32_t PWMIAB;					// SVPWM input ��and ��
	
		 uint32_t RSVD19;
	
	__IO uint32_t PWMMIN;					// SVPWM min sample time
	
	__IO uint32_t PWMDLY;					// SVPWM sample delay
	
		 uint32_t RSVD20;
	
	__IO uint32_t PWMTIM;					// SVPWM period and deadzone time
	
		 uint32_t RSVD21;
	
	__I  uint32_t PWMTSP;					// SVPWM sample point time
	
	__I  uint32_t PWMTAD;					// SVPWM A-phase duty time
	
	__I  uint32_t PWMTBD;					// SVPWM B-phase duty time
	
	__I  uint32_t PWMTCD;					// SVPWM C-phase duty time
} FOC_TypeDef;


#define FOC_CR_MODE_Pos				0		// 0 ��ģ�鵥������ģʽ, 1 FOC��ģ�����ģʽ
#define FOC_CR_MODE_Msk				(0x01 << FOC_CR_MODE_Pos)
#define FOC_CR_RESET_Pos			1
#define FOC_CR_RESET_Msk			(0x01 << FOC_CR_RESET_Pos)
#define FOC_CR_CLARKI3_Pos			4		// CLARK input 3-args mode, 0 ����CLARKIa��CLARKIb��������� 1 ���� ����CLARKIa��CLARKIb��CLARKIc�������
#define FOC_CR_CLARKI3_Msk			(0x01 << FOC_CR_CLARKI3_Pos)
#define FOC_CR_CLARKMOD_Pos			5		// CLARK work mode, 0 �ȷ�ֵ�任
#define FOC_CR_CLARKMOD_Msk			(0x01 << FOC_CR_CLARKMOD_Pos)
#define FOC_CR_PWMMOD_Pos			12		// SVPWM work mode, 0 �߶�ʽģʽ
#define FOC_CR_PWMMOD_Msk			(0x01 << FOC_CR_PWMMOD_Pos)
#define FOC_CR_PWMI2_Pos			13		// SVPWM current sample mode, 0 ���������, 1 ˫�������
#define FOC_CR_PWMI2_Msk			(0x01 << FOC_CR_PWMI2_Pos)

#define FOC_STA_FOC_Pos				0		// д 1 ����һ�� FOC ģ�����, ��������Զ�����
#define FOC_STA_FOC_Msk				(0x01 << FOC_STA_FOC_Pos)
#define FOC_STA_CLARK_Pos			1
#define FOC_STA_CLARK_Msk			(0x01 << FOC_STA_CLARK_Pos)
#define FOC_STA_PARK_Pos			2
#define FOC_STA_PARK_Msk			(0x01 << FOC_STA_PARK_Pos)
#define FOC_STA_PID_Pos				3
#define FOC_STA_PID_Msk				(0x01 << FOC_STA_PID_Pos)
#define FOC_STA_IPARK_Pos			4
#define FOC_STA_IPARK_Msk			(0x01 << FOC_STA_IPARK_Pos)
#define FOC_STA_SVPWM_Pos			5
#define FOC_STA_SVPWM_Msk			(0x01 << FOC_STA_SVPWM_Pos)

#define FOC_IE_FOC_Pos				0		// FOC ģ���������ж�ʹ��
#define FOC_IE_FOC_Msk				(0x01 << FOC_IE_FOC_Pos)
#define FOC_IE_CLARK_Pos			1
#define FOC_IE_CLARK_Msk			(0x01 << FOC_IE_CLARK_Pos)
#define FOC_IE_PARK_Pos				2
#define FOC_IE_PARK_Msk				(0x01 << FOC_IE_PARK_Pos)
#define FOC_IE_PID_Pos				3
#define FOC_IE_PID_Msk				(0x01 << FOC_IE_PID_Pos)
#define FOC_IE_IPARK_Pos			4
#define FOC_IE_IPARK_Msk			(0x01 << FOC_IE_IPARK_Pos)
#define FOC_IE_SVPWM_Pos			5
#define FOC_IE_SVPWM_Msk			(0x01 << FOC_IE_SVPWM_Pos)

#define FOC_SR_BUSY_Pos				0		// ֻ������ FOC->CR.MODE = 1 ʱ
#define FOC_SR_BUSY_Msk				(0x01 << FOC_SR_BUSY_Pos)
#define FOC_SR_FOC_Pos				1		// FOC ģ���������жϱ�־��д 1 ����
#define FOC_SR_FOC_Msk				(0x01 << FOC_SR_FOC_Pos)
#define FOC_SR_CLARK_Pos			2
#define FOC_SR_CLARK_Msk			(0x01 << FOC_SR_CLARK_Pos)
#define FOC_SR_PARK_Pos				3
#define FOC_SR_PARK_Msk				(0x01 << FOC_SR_PARK_Pos)
#define FOC_SR_PID_Pos				4
#define FOC_SR_PID_Msk				(0x01 << FOC_SR_PID_Pos)
#define FOC_SR_IPARK_Pos			5
#define FOC_SR_IPARK_Msk			(0x01 << FOC_SR_IPARK_Pos)
#define FOC_SR_SVPWM_Pos			6
#define FOC_SR_SVPWM_Msk			(0x01 << FOC_SR_SVPWM_Pos)
#define FOC_SR_CLARKOF_Pos			16		// CLARK ģ����������־��д 1 ����
#define FOC_SR_CLARKOF_Msk			(0x01 << FOC_SR_CLARKOF_Pos)
#define FOC_SR_PARKOF_Pos			17
#define FOC_SR_PARKOF_Msk			(0x01 << FOC_SR_PARKOF_Pos)
#define FOC_SR_PIDOF_Pos			18
#define FOC_SR_PIDOF_Msk			(0x01 << FOC_SR_PIDOF_Pos)
#define FOC_SR_IPARKOF_Pos			19
#define FOC_SR_IPARKOF_Msk			(0x01 << FOC_SR_IPARKOF_Pos)
#define FOC_SR_SVPWMOF_Pos			20
#define FOC_SR_SVPWMOF_Msk			(0x01 << FOC_SR_SVPWMOF_Pos)

#define FOC_CLARKIAB_IA_Pos			0
#define FOC_CLARKIAB_IA_Msk			(0xFFFF << FOC_CLARKIAB_IA_Pos)
#define FOC_CLARKIAB_IB_Pos			16
#define FOC_CLARKIAB_IB_Msk			(0xFFFFu<< FOC_CLARKIAB_IB_Pos)

#define FOC_CLARKOAB_OA_Pos			0
#define FOC_CLARKOAB_OA_Msk			(0xFFFF << FOC_CLARKOAB_OA_Pos)
#define FOC_CLARKOAB_OB_Pos			16
#define FOC_CLARKOAB_OB_Msk			(0xFFFFu<< FOC_CLARKOAB_OB_Pos)

#define FOC_PARKIAB_IA_Pos			0
#define FOC_PARKIAB_IA_Msk			(0xFFFF << FOC_PARKIAB_IA_Pos)
#define FOC_PARKIAB_IB_Pos			16
#define FOC_PARKIAB_IB_Msk			(0xFFFFu<< FOC_PARKIAB_IB_Pos)

#define FOC_PARKRAD_SIN_Pos			0
#define FOC_PARKRAD_SIN_Msk			(0xFFFF << FOC_PARKRAD_SIN_Pos)
#define FOC_PARKRAD_COS_Pos			16
#define FOC_PARKRAD_COS_Msk			(0xFFFFu<< FOC_PARKRAD_COS_Pos)

#define FOC_PARKODQ_OD_Pos			0
#define FOC_PARKODQ_OD_Msk			(0xFFFF << FOC_PARKODQ_OD_Pos)
#define FOC_PARKODQ_OQ_Pos			16
#define FOC_PARKODQ_OQ_Msk			(0xFFFFu<< FOC_PARKODQ_OQ_Pos)

#define FOC_PIDIDQ_ID_Pos			0
#define FOC_PIDIDQ_ID_Msk			(0xFFFF << FOC_PIDIDQ_ID_Pos)
#define FOC_PIDIDQ_IQ_Pos			16
#define FOC_PIDIDQ_IQ_Msk			(0xFFFFu<< FOC_PIDIDQ_IQ_Pos)

#define FOC_PIDREF_ID_Pos			0
#define FOC_PIDREF_ID_Msk			(0xFFFF << FOC_PIDREF_ID_Pos)
#define FOC_PIDREF_IQ_Pos			16
#define FOC_PIDREF_IQ_Msk			(0xFFFFu<< FOC_PIDREF_IQ_Pos)

#define FOC_PIDMAX_ID_Pos			0
#define FOC_PIDMAX_ID_Msk			(0xFFFF << FOC_PIDMAX_ID_Pos)
#define FOC_PIDMAX_IQ_Pos			16
#define FOC_PIDMAX_IQ_Msk			(0xFFFFu<< FOC_PIDMAX_IQ_Pos)

#define FOC_PIDMIN_ID_Pos			0
#define FOC_PIDMIN_ID_Msk			(0xFFFF << FOC_PIDMIN_ID_Pos)
#define FOC_PIDMIN_IQ_Pos			16
#define FOC_PIDMIN_IQ_Msk			(0xFFFFu<< FOC_PIDMIN_IQ_Pos)

#define FOC_PIDKP_ID_Pos			0
#define FOC_PIDKP_ID_Msk			(0x3FFF << FOC_PIDKP_ID_Pos)
#define FOC_PIDKP_IQ_Pos			16
#define FOC_PIDKP_IQ_Msk			(0x3FFF << FOC_PIDKP_IQ_Pos)

#define FOC_PIDKI_ID_Pos			0
#define FOC_PIDKI_ID_Msk			(0x3FFF << FOC_PIDKI_ID_Pos)
#define FOC_PIDKI_IQ_Pos			16
#define FOC_PIDKI_IQ_Msk			(0x3FFF << FOC_PIDKI_IQ_Pos)

#define FOC_PIDLERR_ID_Pos			0
#define FOC_PIDLERR_ID_Msk			(0xFFFF << FOC_PIDLERR_ID_Pos)
#define FOC_PIDLERR_IQ_Pos			16
#define FOC_PIDLERR_IQ_Msk			(0xFFFFu<< FOC_PIDLERR_IQ_Pos)

#define FOC_PIDLOUT_OD_Pos			0
#define FOC_PIDLOUT_OD_Msk			(0xFFFF << FOC_PIDLOUT_OD_Pos)
#define FOC_PIDLOUT_OQ_Pos			16
#define FOC_PIDLOUT_OQ_Msk			(0xFFFFu<< FOC_PIDLOUT_OQ_Pos)

#define FOC_PIDODQ_OD_Pos			0
#define FOC_PIDODQ_OD_Msk			(0xFFFF << FOC_PIDODQ_OD_Pos)
#define FOC_PIDODQ_OQ_Pos			16
#define FOC_PIDODQ_OQ_Msk			(0xFFFFu<< FOC_PIDODQ_OQ_Pos)

#define FOC_IPARKIDQ_ID_Pos			0
#define FOC_IPARKIDQ_ID_Msk			(0xFFFF << FOC_IPARKIDQ_ID_Pos)
#define FOC_IPARKIDQ_IQ_Pos			16
#define FOC_IPARKIDQ_IQ_Msk			(0xFFFFu<< FOC_IPARKIDQ_IQ_Pos)

#define FOC_IPARKRAD_SIN_Pos		0
#define FOC_IPARKRAD_SIN_Msk		(0xFFFF << FOC_IPARKRAD_SIN_Pos)
#define FOC_IPARKRAD_COS_Pos		16
#define FOC_IPARKRAD_COS_Msk		(0xFFFFu<< FOC_IPARKRAD_COS_Pos)

#define FOC_IPARKOAB_OA_Pos			0
#define FOC_IPARKOAB_OA_Msk			(0xFFFF << FOC_IPARKOAB_OA_Pos)
#define FOC_IPARKOAB_OB_Pos			16
#define FOC_IPARKOAB_OB_Msk			(0xFFFFu<< FOC_IPARKOAB_OB_Pos)

#define FOC_PWMIAB_IA_Pos			0
#define FOC_PWMIAB_IA_Msk			(0xFFFF << FOC_PWMIAB_IA_Pos)
#define FOC_PWMIAB_IB_Pos			16
#define FOC_PWMIAB_IB_Msk			(0xFFFFu<< FOC_PWMIAB_IB_Pos)

#define FOC_PWMMIN_MIN_Pos			0		// ��С����ʱ��
#define FOC_PWMMIN_MIN_Msk			(0xFFFF << FOC_PWMMIN_MIN_Pos)
#define FOC_PWMMIN_SAMP3_Pos		16		// ˫���������ʱ����3
#define FOC_PWMMIN_SAMP3_Msk		(0xFFFFu<< FOC_PWMMIN_SAMP3_Pos)

#define FOC_PWMDLY_SAMP1_Pos		0		// �����������ʱ����1
#define FOC_PWMDLY_SAMP1_Msk		(0xFFFF << FOC_PWMDLY_SAMP1_Pos)
#define FOC_PWMDLY_SAMP2_Pos		16		// �����������ʱ����2
#define FOC_PWMDLY_SAMP2_Msk		(0xFFFFu<< FOC_PWMDLY_SAMP2_Pos)

#define FOC_PWMTIM_PERIOD_Pos		0		// period
#define FOC_PWMTIM_PERIOD_Msk		(0xFFFF << FOC_PWMTIM_PERIOD_Pos)
#define FOC_PWMTIM_DZ_Pos			16		// deadzone
#define FOC_PWMTIM_DZ_Msk			(0xFFFFu<< FOC_PWMTIM_DZ_Pos)

#define FOC_PWMTSP_POINT1_Pos		0		// ����ʱ������1
#define FOC_PWMTSP_POINT1_Msk		(0xFFFF << FOC_PWMTSP_POINT1_Pos)
#define FOC_PWMTSP_POINT2_Pos		16
#define FOC_PWMTSP_POINT2_Msk		(0xFFFFu<< FOC_PWMTSP_POINT2_Pos)

#define FOC_PWMAD_DUTYA_Pos			0
#define FOC_PWMAD_DUTYA_Msk			(0xFFFF << FOC_PWMAD_DUTYA_Pos)
#define FOC_PWMAD_DUTYAN_Pos		16
#define FOC_PWMAD_DUTYAN_Msk		(0xFFFFu<< FOC_PWMAD_DUTYAN_Pos)

#define FOC_PWMBD_DUTYB_Pos			0
#define FOC_PWMBD_DUTYB_Msk			(0xFFFF << FOC_PWMBD_DUTYB_Pos)
#define FOC_PWMBD_DUTYBN_Pos		16
#define FOC_PWMBD_DUTYBN_Msk		(0xFFFFu<< FOC_PWMBD_DUTYBN_Pos)

#define FOC_PWMCD_DUTYC_Pos			0
#define FOC_PWMCD_DUTYC_Msk			(0xFFFF << FOC_PWMCD_DUTYC_Pos)
#define FOC_PWMCD_DUTYCN_Pos		16
#define FOC_PWMCD_DUTYCN_Msk		(0xFFFFu<< FOC_PWMCD_DUTYCN_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t ADDR;
	
	__IO uint32_t ERASE;
	
	__IO uint32_t RSVD;
	
	__IO uint32_t CFG0;
	
	__IO uint32_t CFG1;						//����д�� 0x5A5A5A5A��0xA5A5A5A5 ��� readonly��д����������ֵ�ָ� readonly
	
	__IO uint32_t CFG2;
	
	__IO uint32_t CFG3;
	
	__IO uint32_t CFG4;
	
	__IO uint32_t STAT;
	
	__IO uint32_t REMAP;
} FMC_TypeDef;


#define FMC_ERASE_ADDR_Pos			0		//512 Byte per Page
#define FMC_ERASE_ADDR_Msk			(0x1FFFF<< FMC_ERASE_ADDR_Pos)
#define FMC_ERASE_REQ_Pos			24
#define FMC_ERASE_REQ_Msk			(0xFFu<< FMC_ERASE_REQ_Pos)

#define FMC_CFG0_WREN_Pos			0
#define FMC_CFG0_WREN_Msk			(0x01 << FMC_CFG0_WREN_Pos)

#define FMC_STAT_ERASEBUSY_Pos		0
#define FMC_STAT_ERASEBUSY_Msk		(0x01 << FMC_STAT_ERASEBUSY_Pos)
#define FMC_STAT_PROGBUSY_Pos		1
#define FMC_STAT_PROGBUSY_Msk		(0x01 << FMC_STAT_PROGBUSY_Pos)
#define FMC_STAT_READBUSY_Pos		2
#define FMC_STAT_READBUSY_Msk		(0x01 << FMC_STAT_READBUSY_Pos)
#define FMC_STAT_FIFOEMPTY_Pos		3		//Write FIFO Empty
#define FMC_STAT_FIFOEMPTY_Msk		(0x01 << FMC_STAT_FIFOEMPTY_Pos)
#define FMC_STAT_FIFOFULL_Pos		4		//Write FIFO Full
#define FMC_STAT_FIFOFULL_Msk		(0x01 << FMC_STAT_FIFOFULL_Pos)
#define FMC_STAT_READONLY_Pos		7
#define FMC_STAT_READONLY_Msk		(0x01 << FMC_STAT_READONLY_Pos)
#define FMC_STAT_INITDONE_Pos		30
#define FMC_STAT_INITDONE_Msk		(0x01 << FMC_STAT_INITDONE_Pos)
#define FMC_STAT_IDLE_Pos			31
#define FMC_STAT_IDLE_Msk			(0x01u<< FMC_STAT_IDLE_Pos)

#define FMC_REMAP_ON_Pos			0
#define FMC_REMAP_ON_Msk			(0x01 << FMC_REMAP_ON_Pos)
#define FMC_REMAP_OFFSET_Pos		1		//��0x000-0x800��2K��ַ�ķ���ӳ�䵽2K*OFFSET-2K*(OFFSET+1)��ַ��
#define FMC_REMAP_OFFSET_Msk		(0x0F << FMC_REMAP_OFFSET_Pos)




typedef struct {
	__IO uint32_t RSTVAL;					//��������������ֵʱ������λ
	
	__IO uint32_t INTVAL;					//��������������ֵʱ�����ж�
	
	__IO uint32_t CR;
	
	__IO uint32_t IF;						//[0] �жϱ�־��д1����
	
	__IO uint32_t FEED;						//д0x55ι��
} WDT_TypeDef;


#define WDT_CR_EN_Pos				0
#define WDT_CR_EN_Msk				(0x01 << WDT_CR_EN_Pos)
#define WDT_CR_RSTEN_Pos			1
#define WDT_CR_RSTEN_Msk			(0x01 << WDT_CR_RSTEN_Pos)
#define WDT_CR_INTEN_Pos			2
#define WDT_CR_INTEN_Msk			(0x01 << WDT_CR_INTEN_Pos)
#define WDT_CR_WINEN_Pos			3		//Window function enable
#define WDT_CR_WINEN_Msk			(0x01 << WDT_CR_WINEN_Pos)
#define WDT_CR_CLKDIV_Pos			8		//WDT����ʱ�ӷ�Ƶֵ = pow(2, CLKDIV+1)
#define WDT_CR_CLKDIV_Msk			(0x0F << WDT_CR_CLKDIV_Pos)




/******************************************************************************/
/*						 Peripheral memory map							  */
/******************************************************************************/
#define RAM_BASE		   	0x20000000
#define AHB_BASE			0x40000000
#define APB1_BASE		 	0x40040000


/* AHB Peripheral memory map */
#define SYS_BASE			(AHB_BASE + 0x00000)

#define DMA_BASE			(AHB_BASE + 0x00800)

#define DIV_BASE			(AHB_BASE + 0x01800)

#define GPIOA_BASE			(AHB_BASE + 0x02000)
#define GPIOB_BASE			(AHB_BASE + 0x02800)


/* APB1 Peripheral memory map */
#define UART0_BASE			(APB1_BASE + 0x0000)
#define UART1_BASE			(APB1_BASE + 0x0800)

#define SPI0_BASE			(APB1_BASE + 0x1000)

#define PWM0_BASE			(APB1_BASE + 0x1800)
#define PWM1_BASE			(APB1_BASE + 0x1880)
#define PWMG_BASE			(APB1_BASE + 0x1C00)

#define TIMR0_BASE			(APB1_BASE + 0x2000)
#define TIMRG_BASE			(APB1_BASE + 0x2400)

#define BTIMR0_BASE			(APB1_BASE + 0x2800)
#define BTIMR1_BASE			(APB1_BASE + 0x2840)
#define BTIMR2_BASE			(APB1_BASE + 0x2880)
#define BTIMRG_BASE			(APB1_BASE + 0x2C00)

#define FOC_BASE			(APB1_BASE + 0x3000)

#define ADC0_BASE			(APB1_BASE + 0x3800)

#define FMC_BASE			(APB1_BASE + 0x4000)		//Flash Memory Controller

#define PORTA_BASE			(APB1_BASE + 0x5000)
#define PORTB_BASE			(APB1_BASE + 0x5010)

#define WDT_BASE			(APB1_BASE + 0x5800)


/******************************************************************************/
/*						 Peripheral declaration							 */
/******************************************************************************/
#define SYS					((SYS_TypeDef  *) SYS_BASE)

#define PORTA				((PORT_TypeDef *) PORTA_BASE)
#define PORTB				((PORT_TypeDef *) PORTB_BASE)

#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB				((GPIO_TypeDef *) GPIOB_BASE)

#define TIMR0				((TIMR_TypeDef *) TIMR0_BASE)
#define TIMRG				((TIMRG_TypeDef*) TIMRG_BASE)

#define BTIMR0				((TIMR_TypeDef *) BTIMR0_BASE)
#define BTIMR1				((TIMR_TypeDef *) BTIMR1_BASE)
#define BTIMR2				((TIMR_TypeDef *) BTIMR2_BASE)
#define BTIMRG				((TIMRG_TypeDef*) BTIMRG_BASE)

#define UART0				((UART_TypeDef *) UART0_BASE)
#define UART1				((UART_TypeDef *) UART1_BASE)

#define SPI0				((SPI_TypeDef  *) SPI0_BASE)

#define ADC0 				((ADC_TypeDef  *) ADC0_BASE)

#define PWM0				((PWM_TypeDef  *) PWM0_BASE)
#define PWM1				((PWM_TypeDef  *) PWM1_BASE)
#define PWMG				((PWMG_TypeDef *) PWMG_BASE)

#define DIV					((DIV_TypeDef  *) DIV_BASE)

#define FOC					((FOC_TypeDef  *) FOC_BASE)

#define DMA					((DMA_TypeDef  *) DMA_BASE)

#define FMC					((FMC_TypeDef  *) FMC_BASE)

#define WDT					((WDT_TypeDef  *) WDT_BASE)


#include "SWM231_port.h"
#include "SWM231_gpio.h"
#include "SWM231_exti.h"
#include "SWM231_timr.h"
#include "SWM231_uart.h"
#include "SWM231_spi.h"
#include "SWM231_pwm.h"
#include "SWM231_adc.h"
#include "SWM231_dma.h"
#include "SWM231_div.h"
#include "SWM231_foc.h"
#include "SWM231_wdt.h"
#include "SWM231_flash.h"
#include "SWM231_iofilt.h"



#ifdef  SW_LOG_RTT
#define log_printf(...)	 	SEGGER_RTT_printf(0, __VA_ARGS__)
#else
#define log_printf(...)	 	printf(__VA_ARGS__)
#endif


#ifndef SW_LOG_LEVEL
#define SW_LOG_LEVEL        0
#endif

#if (SW_LOG_LEVEL > 0)
#define SW_LOG_ERR(...)    	{						 \
							log_printf("ERROR: ");   \
							log_printf(__VA_ARGS__); \
							log_printf("\n");		 \
							}

#if (SW_LOG_LEVEL > 1)
#define SW_LOG_WARN(...) 	{						 \
							log_printf("WARN : ");   \
							log_printf(__VA_ARGS__); \
							log_printf("\n");		 \
							}

#if (SW_LOG_LEVEL > 2)
#define SW_LOG_INFO(...)   	{						 \
							log_printf("INFO : ");   \
							log_printf(__VA_ARGS__); \
							log_printf("\n");		 \
							}
#else
#define SW_LOG_INFO(...)
#endif

#else
#define SW_LOG_WARN(...)
#define SW_LOG_INFO(...)
#endif

#else
#define SW_LOG_ERR(...)
#define SW_LOG_WARN(...)
#define SW_LOG_INFO(...)
#endif


#endif //__SWM231_H__
