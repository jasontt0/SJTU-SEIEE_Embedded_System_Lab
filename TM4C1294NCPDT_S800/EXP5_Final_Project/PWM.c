//*****************************************************************************
//
// PWM.c - API for PWM.
//
// Copyright��2020-2021,�Ϻ���ͨ��ѧ���ӹ���ϵʵ���ѧ����
// 
// Author:	�Ϻ���ͨ��ѧ���ӹ���ϵʵ���ѧ����
// Version: 1.0.0.20210508 
// Date��2021-05-08
// History��
//
//*****************************************************************************

#include "PWM.h"
extern uint32_t ui32SysClock;    // ϵͳʱ��
//extern uint32_t ui32SysClock;    // ϵͳʱ��

//*******************************************************************************************************
// 
// ����ԭ�ͣ�void PWMInit()
// �������ܣ���������PG0ʹ�ø��ù���M0PWM4
// ������������
// ��������ֵ����
//
//*******************************************************************************************************
void PWMInit()
{
    //PF3************************************************************
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // PWM0ʹ��

	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true); // ʹ��(����)PWM0_7�����
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);				// ʹ��PWM0ģ���3�ŷ�����(��Ϊ7��PWM��3�ŷ�����������)
	// PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32SysClock / ui32Freq_Hz); // ����Freq_Hz����PWM����

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK); // ʹ��GPIOF
	GPIOPinConfigure(GPIO_PK5_M0PWM7);			 // �������Ÿ���
	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5); // ����ӳ��

	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // ����PWM������
	// PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,(PWMGenPeriodGet(PWM0_BASE, PWM_GEN_2)/ 2)); //����ռ�ձ�Ϊ50%
}

//*******************************************************************************************************
// 
// ����ԭ�ͣ�void PWMStart(uint32_t ui32Freq_Hz)
// �������ܣ�����Ƶ��Ϊui32Freq_Hz�ķ���(ռ�ձ�Ϊ50%��PWM)���������ΪM0PWM4(PG0)
//          �ú�����Ϊ�˷����û�û���źŷ�����ʱ���������źŶ���д�ġ�
// ����������ui32Freq_Hz ��Ҫ�����ķ�����Ƶ��
// ��������ֵ����
//
//*******************************************************************************************************
void PWMStart(uint32_t ui32Freq_Hz)//PK5
{

    PWMGenDisable(PWM0_BASE, PWM_GEN_3); // ֹͣ����PWM�ź�
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 1)); // ����ռ�ձ�Ϊ100%(��Ƶ��)

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32SysClock / ui32Freq_Hz);					 // ����Freq_Hz����PWM����
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 2)); // ����ռ�ձ�Ϊ50%

	PWMGenEnable(PWM0_BASE, PWM_GEN_3); // ʹ��PWM0ģ���3�ŷ�����(��Ϊ7��PWM��3�ŷ�����������)
}
//*******************************************************************************************************






//*******************************************************************************************************
// 
// ����ԭ�ͣ�void PWMStop()
// �������ܣ�M0PWM7(PK5)ֹͣ����PWM�ź�
// ������������
// ��������ֵ����
//
//*******************************************************************************************************
void PWMStop()
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 1)); // ����ռ�ձ�Ϊ50%����Ƶ�ʣ�
    PWMGenDisable(PWM0_BASE, PWM_GEN_3);   // M0PWM7(PK5)ֹͣ����PWM�ź�

}

//***********************************************************8
