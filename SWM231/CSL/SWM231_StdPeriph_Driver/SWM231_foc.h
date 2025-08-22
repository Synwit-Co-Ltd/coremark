#ifndef __SWM231_FOC_H__
#define __SWM231_FOC_H__


typedef struct {
	uint8_t  Mode;				// FOC_MODE_ALL, FOC_MODE_PART
	
	uint8_t  Clark3Input;		// 0 Clark ��������������ģʽ��CLARKIa��CLARKIb����1 Clark ��������������ģʽ��CLARKIa��CLARKIb��CLARKIc��
	
	uint8_t  SVPWM2Resistor;	// 0 SVPWM ʹ�õ����������1 SVPWM ʹ��˫�������
	
	uint8_t  INTEn;				// interrupt enable����Чֵ�� FOC_IT_FOC��FOC_IT_CLARK��FOC_IT_PARK��FOC_IT_PID��FOC_IT_iPARK��FOC_IT_SVPWM ���䡰��
} FOC_InitStructure;


#define FOC_MODE_PART	0		// CLARK��PARK��PID��iPARK��SVPWM ����ͨ�� FOC->STA �еĶ�Ӧλ����
#define FOC_MODE_ALL	1		// ���й���ͨ�� FOC->STA.FOC ����λ��������


/* Interrupt Type */
#define FOC_IT_FOC		FOC_IE_FOC_Msk
#define FOC_IT_CLARK	FOC_IE_CLARK_Msk
#define FOC_IT_PARK		FOC_IE_PARK_Msk
#define FOC_IT_PID		FOC_IE_PID_Msk
#define FOC_IT_iPARK	FOC_IE_IPARK_Msk
#define FOC_IT_SVPWM	FOC_IE_SVPWM_Msk



void FOC_Init(FOC_TypeDef * FOCx, FOC_InitStructure * initStruct);


static __INLINE void FOC_Clark_Input(int16_t ia, int16_t ib)
{
	FOC->CLARKIAB = ((uint16_t)ia << FOC_CLARKIAB_IA_Pos) |
					((uint16_t)ib << FOC_CLARKIAB_IB_Pos);
}

static __INLINE void FOC_Clark_Input3(int16_t ia, int16_t ib, int16_t ic)
{
	FOC->CLARKIAB = ((uint16_t)ia << FOC_CLARKIAB_IA_Pos) |
					((uint16_t)ib << FOC_CLARKIAB_IB_Pos);
	
	FOC->CLARKIC = ic;
}

static __INLINE void FOC_Clark_Start(void)
{
	FOC->STA |= FOC_STA_CLARK_Msk;
}

static __INLINE uint32_t FOC_Clark_Busy(void)
{
	return FOC->STA & FOC_STA_CLARK_Msk;
}

static __INLINE uint32_t FOC_Clark_Error(void)
{
	uint32_t overflow = FOC->SR & FOC_SR_CLARKOF_Msk;
	
	FOC->SR = overflow;
	
	return overflow;
}

static __INLINE void FOC_Clark_Result(int16_t * alpha, int16_t * beta)
{
	*alpha = FOC->CLARKOAB & FOC_CLARKOAB_OA_Msk;
	*beta = (FOC->CLARKOAB & FOC_CLARKOAB_OB_Msk) >> FOC_CLARKOAB_OB_Pos;
}


static __INLINE void FOC_Park_Input(int16_t alpha, int16_t beta, int16_t sin, int16_t cos)
{
	FOC->PARKIAB = ((uint16_t)alpha << FOC_PARKIAB_IA_Pos) |
				   ((uint16_t)beta  << FOC_PARKIAB_IB_Pos);
	
	FOC->PARKRAD = ((uint16_t)sin << FOC_PARKRAD_SIN_Pos) |
				   ((uint16_t)cos << FOC_PARKRAD_COS_Pos);
}

static __INLINE void FOC_Park_Start(void)
{
	FOC->STA |= FOC_STA_PARK_Msk;
}

static __INLINE uint32_t FOC_Park_Busy(void)
{
	return FOC->STA & FOC_STA_PARK_Msk;
}

static __INLINE uint32_t FOC_Park_Error(void)
{
	uint32_t overflow = FOC->SR & FOC_SR_PARKOF_Msk;
	
	FOC->SR = overflow;
	
	return overflow;
}

static __INLINE void FOC_Park_Result(int16_t * id, int16_t * iq)
{
	*id = FOC->PARKODQ & FOC_PARKODQ_OD_Msk;
	*iq =(FOC->PARKODQ & FOC_PARKODQ_OQ_Msk) >> FOC_PARKODQ_OQ_Pos;
}


static __INLINE void FOC_iPark_Input(int16_t vd, int16_t vq, int16_t sin, int16_t cos)
{
	FOC->IPARKIDQ = ((uint16_t)vd << FOC_IPARKIDQ_ID_Pos) |
					((uint16_t)vq << FOC_IPARKIDQ_IQ_Pos);
	
	FOC->IPARKRAD = ((uint16_t)sin << FOC_IPARKRAD_SIN_Pos) |
					((uint16_t)cos << FOC_IPARKRAD_COS_Pos);
}

static __INLINE void FOC_iPark_Start(void)
{
	FOC->STA |= FOC_STA_IPARK_Msk;
}

static __INLINE uint32_t FOC_iPark_Busy(void)
{
	return FOC->STA & FOC_STA_IPARK_Msk;
}

static __INLINE uint32_t FOC_iPark_Error(void)
{
	uint32_t overflow = FOC->SR & FOC_SR_IPARKOF_Msk;
	
	FOC->SR = overflow;
	
	return overflow;
}

static __INLINE void FOC_iPark_Result(int16_t * alpha, int16_t * beta)
{
	*alpha = FOC->IPARKOAB & FOC_IPARKOAB_OA_Msk;
	*beta = (FOC->IPARKOAB & FOC_IPARKOAB_OB_Msk) >> FOC_IPARKOAB_OB_Pos;
}


// it: interrupt type����Чֵ�� FOC_IT_FOC��FOC_IT_CLARK��FOC_IT_PARK��FOC_IT_PID��FOC_IT_iPARK��FOC_IT_SVPWM ���䡰��
static __INLINE void FOC_INTEn(uint32_t it)
{
	FOC->IE |= it;
}

static __INLINE void FOC_INTDis(uint32_t it)
{
	FOC->IE &= ~it;
}

static __INLINE void FOC_INTClr(uint32_t it)
{
	FOC->SR = (it << 1);
}

static __INLINE uint32_t FOC_INTStat(uint32_t it)
{
	return (FOC->SR & (it << 1));
}

#endif //__SWM231_FOC_H__
