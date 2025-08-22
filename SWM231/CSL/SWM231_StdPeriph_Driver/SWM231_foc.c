/****************************************************************************************************************************************** 
* �ļ�����: SWM231_foc.c
* ����˵��:	SWM231��Ƭ����FOCģ��������
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
* �汾����:	V1.0.0		2016��1��30��
* ������¼:  
*
*
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology
*******************************************************************************************************************************************/
#include "SWM231.h"
#include "SWM231_foc.h"


/****************************************************************************************************************************************** 
* ��������:	FOC_Init()
* ����˵��:	FOCģ���ʼ��
* ��    ��: FOC_TypeDef * FOCx	ָ��Ҫ�����õ�FOCģ�飬��Чֵ����FOC
*			FOC_InitStructure * initStruct    ����FOCģ������趨ֵ�Ľṹ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void FOC_Init(FOC_TypeDef * FOCx, FOC_InitStructure * initStruct)
{
	SYS->CLKEN0 |= SYS_CLKEN0_FOC_Msk;
	
	for(int i = 0; i < 3; i++) __NOP();
	
	FOCx->CR = (initStruct->Mode		   << FOC_CR_MODE_Pos) |
			   (initStruct->Clark3Input	   << FOC_CR_CLARKI3_Pos) |
			   (initStruct->SVPWM2Resistor << FOC_CR_PWMI2_Pos);
	
	FOC_INTClr(initStruct->INTEn);
	FOC_INTEn(initStruct->INTEn);
}

