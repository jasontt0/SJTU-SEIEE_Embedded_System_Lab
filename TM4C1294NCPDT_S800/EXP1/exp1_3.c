//*****************************************************************************
//
// Copyright: 2020-2021, �Ϻ���ͨ��ѧ���ӹ���ϵʵ���ѧ����
// File name: exp2_0.c
// Description:
//    1.������λ�󣬵װ����ұ�4λ������Զ���ʾ��ʱ��ֵ�����λ��Ӧ��λ��0.1�룻
//    2.������λ�󣬵װ���8��LED�����������ʽ��������ѭ���任��Լ0.5��任1�Σ�
//    3.��û�а�������ʱ����������ڶ�λ�������ʾ��0����
//      ���˹�����ĳ�����������ʾ�ü��ı�ţ�
//      �˿���λ��ʱ�������ͣ�仯��ֹͣ��ʱ��ֱ���ſ��������Զ�������ʱ��
// Author:	�Ϻ���ͨ��ѧ���ӹ���ϵʵ���ѧ����
// Version: 1.0.0.20201228
// Date��2020-12-28
// History��
//
//*****************************************************************************

//*****************************************************************************
//
// ͷ�ļ�
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // ��ַ�궨��
#include "inc/hw_types.h"         // �������ͺ궨�壬�Ĵ������ʺ���
#include "driverlib/debug.h"      // ������
#include "driverlib/gpio.h"       // ͨ��IO�ں궨��
#include "driverlib/pin_map.h"    // TM4Cϵ��MCU��Χ�豸�ܽź궨��
#include "driverlib/sysctl.h"	  // ϵͳ���ƶ���
#include "driverlib/systick.h"    // SysTick Driver ԭ��
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver ԭ��

//#include "tm1638.h"               // �����TM1638оƬ�йصĺ���

//#include "driverlib/uart.h"       // UART��غ궨��
//#include "utils/uartstdio.h"      // UART0��Ϊ����̨��غ���ԭ������

//*****************************************************************************
//
// �궨��
//
//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTickƵ��Ϊ50Hz����ѭ����ʱ����20ms

#define V_T100ms	5                   // 0.1s�����ʱ�����ֵ��5��20ms
#define V_T500ms	25                  // 0.5s�����ʱ�����ֵ��25��20ms

//*****************************************************************************
//
// ����ԭ������
//
//*****************************************************************************
void GPIOInit(void);        // GPIO��ʼ��
void SysTickInit(void);     // ����SysTick�ж�
void DevicesInit(void);     // MCU������ʼ����ע���������������

//void counter_for_100ms(void);	//����1��Ƶ��Ϊ100ms�ļ�ʱ����ʾ
//void flash_light_run_right(void);	//����2������ƣ���������
//void flash_light_run_left(void);	//����2������ƣ���������
//void show_key_number(void);	//����3����ʾ��ֵ

void switch_mode(void);	//ת�䲻ͬ����ģʽ
void run_mode(void);	//���в�ͬ�Ĺ���ģʽ

void FLASH_PF0(void);
void FLASH_PF4(void);


//*****************************************************************************
//
// ��������
//
//*****************************************************************************

// �����ʱ������
uint8_t clock100ms = 0;
//uint8_t clock500ms = 0;


// �����ʱ�������־
uint8_t clock100ms_flag = 0;
//uint8_t clock500ms_flag = 0;

// �����ü�����
//uint32_t test_counter = 0;

// 8λ�������ʾ�����ֻ���ĸ����
// ע����������λ�������������Ϊ4��5��6��7��0��1��2��3
//uint8_t digit[8]= {' ',' ',' ',' ','_',' ','_',' '};

// 8λС���� 1��  0��
// ע����������λС����������������Ϊ4��5��6��7��0��1��2(��)��3
//uint8_t pnt = 0x04;//4=>������2��λΪ1������Ϊ0

// 8��LEDָʾ��״̬��0��1��
// ע������ָʾ�ƴ������������Ϊ7��6��5��4��3��2��1��0
//     ��ӦԪ��LED8��LED7��LED6��LED5��LED4��LED3��LED2��LED1
//uint8_t led[] = {1, 1, 1, 1, 1, 1, 1, 0};

// ��ǰ����ֵ
//uint8_t key_code = 0;

// ϵͳʱ��Ƶ��
uint32_t ui32SysClock;

//ָʾ����ģʽ
uint8_t mode;

uint8_t light_flag=0;

uint8_t current_PJ=1;	//��ȡPJ0����
uint8_t previous_PJ=1;	//PJ0ǰһ״̬


//*****************************************************************************
//
// ������
//
//*****************************************************************************
int main(void)
{

  DevicesInit();            //  MCU������ʼ��

  while (clock100ms < 3);   // ��ʱ>60ms,�ȴ�TM1638�ϵ����
  //TM1638_Init();	          // ��ʼ��TM1638

	mode=1;
  
	while (1)
	{
		run_mode();		
	}
	
}

//*****************************************************************************
//
// ����ԭ�ͣ�void GPIOInit(void)
// �������ܣ�GPIO��ʼ����ʹ��PortK������PK4,PK5Ϊ�����ʹ��PortM������PM0Ϊ�����
//          ��PK4����TM1638��STB��PK5����TM1638��DIO��PM0����TM1638��CLK��
// ������������
// ��������ֵ����
//
//*****************************************************************************
void GPIOInit(void)
{

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		  // ʹ�ܶ˿� F
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}; // �ȴ��˿� F׼�����
		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);		   // ʹ�ܶ˿� J
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {}; // �ȴ��˿� J׼�����

  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
		
	// ���ö˿� J�ĵ�0λ��PJ0��Ϊ��������
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
	// �˿� J�ĵ�0λ��Ϊ�������룬�������óɡ�����������
  GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		
}

//*****************************************************************************
//
// ����ԭ�ͣ�SysTickInit(void)
// �������ܣ�����SysTick�ж�
// ������������
// ��������ֵ����
//
//*****************************************************************************
void SysTickInit(void)
{
  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // ������������,��ʱ����20ms=1/50s
  SysTickEnable();  			// SysTickʹ��
  SysTickIntEnable();			// SysTick�ж�����
}

//*****************************************************************************
//
// ����ԭ�ͣ�void DevicesInit(void)
// �������ܣ�CU������ʼ��������ϵͳʱ�����á�GPIO��ʼ����SysTick�ж�����
// ������������
// ��������ֵ����
//
//*****************************************************************************
void DevicesInit(void)
{
  // ʹ���ⲿ25MHz��ʱ��Դ������PLL��Ȼ���ƵΪ20MHz
  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
                                     SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480),
                                    20000000);

  GPIOInit();             // GPIO��ʼ��
  SysTickInit();          // ����SysTick�ж�
  IntMasterEnable();		// ���ж�����
}

//*****************************************************************************
//
// ����ԭ�ͣ�void SysTick_Handler(void)
// �������ܣ�SysTick�жϷ������
// ������������
// ��������ֵ����
//
//*****************************************************************************
void SysTick_Handler(void)       // ��ʱ����Ϊ20ms,��ÿ20ms�Զ�����һ�η�����
{
  // 0.1������ʱ������
  if (++clock100ms >= V_T100ms)
    {
      clock100ms_flag = 1;    // ��0.1�뵽ʱ�������־��1
      clock100ms = 0;
    }

		
		current_PJ=GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0);
		
		if(previous_PJ==1 && current_PJ==0) switch_mode();
		previous_PJ=current_PJ;
  // 0.5������ʱ������
  /*if (++clock500ms >= V_T500ms)
    {
      clock500ms_flag = 1;    // ��0.5�뵽ʱ�������־��1
      clock500ms = 0;
    }

  // ˢ��ȫ������ܺ�LEDָʾ��
  TM1638_RefreshDIGIandLED(digit, pnt, led);

	show_key_number();//����3����
		*/
	
}



//****************************************************************************
//����1
void FLASH_PF0(void)
{
	if (clock100ms_flag == 1)      // ���0.1�붨ʱ�Ƿ�
        {
          clock100ms_flag		= 0;
					if(light_flag==0)
					{
						light_flag=1;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);        // �ر� LED4(D4-PF0)

					}
					else
					{
						light_flag=0;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);        // �ر� LED4(D4-PF0)
					}
        }
}

//*************************************************************
void FLASH_PF4(void)
{
	if (clock100ms_flag == 1)      // ���0.1�붨ʱ�Ƿ�
        {
          clock100ms_flag		= 0;
					if(light_flag==0)
					{
						light_flag=1;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);        // �ر� LED4(D4-PF0)

					}
					else
					{
						light_flag=0;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);        // �ر� LED4(D4-PF0)
					}
        }
}

/*
//****************************************************
//����2,��������
void flash_light_run_right(void)
{
	uint8_t temp,i;
	
	if (clock500ms_flag == 1)   // ���0.5�붨ʱ�Ƿ�
        {
          clock500ms_flag = 0;
          // 8��ָʾ��������Ʒ�ʽ��ÿ0.5�����ң�ѭ�����ƶ�һ��
          temp = led[0];
          for (i = 0; i < 7; i++) led[i] = led[i + 1];//ÿ20ms������������forѭ��һ�Σ�forѭ���������
          led[7] = temp;
        }
}


//***********************************
//����2����������
void flash_light_run_left(void)
{
	uint8_t temp,i;
	
	if (clock500ms_flag == 1)   // ���0.5�붨ʱ�Ƿ�
		{
          clock500ms_flag = 0;
          // 8��ָʾ��������Ʒ�ʽ��ÿ0.5�����ң�ѭ�����ƶ�һ��
          temp = led[7];
          for (i = 7; i >0; i--) led[i] = led[i - 1];//ÿ20ms������������forѭ��һ�Σ�forѭ���������
          led[0] = temp;
    }

}

		
//***********************************************************
//����3
void show_key_number(void)
{
	
	// ��鵱ǰ�������룬0�����޼�������1-9��ʾ�ж�Ӧ����
  // ������ʾ��һλ�������
  key_code = TM1638_Readkeyboard();

  digit[5] = key_code;//5��λ����ʾ������
	
}
*/

//**********************************
//ת�乤��ģʽ
void switch_mode(void)
{
	switch(mode)
	{
		case 1:
		{
			mode=2;
			break;
		}
		case 2:
		{
			mode=3;
			break;
		}
		case 3:
		{
			mode=4;
			break;
		}
		case 4:
		{
			mode=1;
			break;
		}		
	}
}


//*************************************************
//���е�ǰ����ģʽ
void run_mode(void)
{
	
	if(mode==1) FLASH_PF0();
	if(mode==2) GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);        // �ر� LED4(D4-PF0)
	if(mode==3) FLASH_PF4();
	if(mode==4) GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);        // �ر� LED4(D4-PF0)


	
}

