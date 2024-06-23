//*****************************************************************************
//
// Copyright: 2020-2021, 上海交通大学电子工程系实验教学中心
// File name: exp2_0.c
// Description:
//    1.开机或复位后，底板上右边4位数码管自动显示计时数值，最低位对应单位是0.1秒；
//    2.开机或复位后，底板上8个LED灯以跑马灯形式由左向右循环变换，约0.5秒变换1次；
//    3.当没有按键按下时，从左边数第二位数码管显示“0”；
//      当人工按下某键，数码管显示该键的编号；
//      此刻四位计时数码管暂停变化，停止计时，直到放开按键后自动继续计时。
// Author:	上海交通大学电子工程系实验教学中心
// Version: 1.0.0.20201228
// Date：2020-12-28
// History：
//
//*****************************************************************************

//*****************************************************************************
//
// 头文件
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"        // 基址宏定义
#include "inc/hw_types.h"         // 数据类型宏定义，寄存器访问函数
#include "driverlib/debug.h"      // 调试用
#include "driverlib/gpio.h"       // 通用IO口宏定义
#include "driverlib/pin_map.h"    // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"	  // 系统控制定义
#include "driverlib/systick.h"    // SysTick Driver 原型
#include "driverlib/interrupt.h"  // NVIC Interrupt Controller Driver 原型

//#include "tm1638.h"               // 与控制TM1638芯片有关的函数

//#include "driverlib/uart.h"       // UART相关宏定义
//#include "utils/uartstdio.h"      // UART0作为控制台相关函数原型声明

//*****************************************************************************
//
// 宏定义
//
//*****************************************************************************
#define SYSTICK_FREQUENCY		50		// SysTick频率为50Hz，即循环定时周期20ms

#define V_T100ms	5                   // 0.1s软件定时器溢出值，5个20ms
#define V_T500ms	25                  // 0.5s软件定时器溢出值，25个20ms

//*****************************************************************************
//
// 函数原型声明
//
//*****************************************************************************
void GPIOInit(void);        // GPIO初始化
void SysTickInit(void);     // 设置SysTick中断
void DevicesInit(void);     // MCU器件初始化，注：会调用上述函数

//void counter_for_100ms(void);	//功能1：频率为100ms的计时和显示
//void flash_light_run_right(void);	//功能2：走马灯，从左向右
//void flash_light_run_left(void);	//功能2：走马灯，从右向左
//void show_key_number(void);	//功能3：显示键值

void switch_mode(void);	//转变不同工作模式
void run_mode(void);	//运行不同的工作模式

void FLASH_PF0(void);
void FLASH_PF4(void);


//*****************************************************************************
//
// 变量定义
//
//*****************************************************************************

// 软件定时器计数
uint8_t clock100ms = 0;
//uint8_t clock500ms = 0;


// 软件定时器溢出标志
uint8_t clock100ms_flag = 0;
//uint8_t clock500ms_flag = 0;

// 测试用计数器
//uint32_t test_counter = 0;

// 8位数码管显示的数字或字母符号
// 注：板上数码位从左到右序号排列为4、5、6、7、0、1、2、3
//uint8_t digit[8]= {' ',' ',' ',' ','_',' ','_',' '};

// 8位小数点 1亮  0灭
// 注：板上数码位小数点从左到右序号排列为4、5、6、7、0、1、2(亮)、3
//uint8_t pnt = 0x04;//4=>二进制2号位为1，其余为0

// 8个LED指示灯状态，0灭，1亮
// 注：板上指示灯从左到右序号排列为7、6、5、4、3、2、1、0
//     对应元件LED8、LED7、LED6、LED5、LED4、LED3、LED2、LED1
//uint8_t led[] = {1, 1, 1, 1, 1, 1, 1, 0};

// 当前按键值
//uint8_t key_code = 0;

// 系统时钟频率
uint32_t ui32SysClock;

//指示工作模式
uint8_t mode;

uint8_t light_flag=0;

uint8_t current_PJ=1;	//读取PJ0输入
uint8_t previous_PJ=1;	//PJ0前一状态


//*****************************************************************************
//
// 主程序
//
//*****************************************************************************
int main(void)
{

  DevicesInit();            //  MCU器件初始化

  while (clock100ms < 3);   // 延时>60ms,等待TM1638上电完成
  //TM1638_Init();	          // 初始化TM1638

	mode=1;
  
	while (1)
	{
		run_mode();		
	}
	
}

//*****************************************************************************
//
// 函数原型：void GPIOInit(void)
// 函数功能：GPIO初始化。使能PortK，设置PK4,PK5为输出；使能PortM，设置PM0为输出。
//          （PK4连接TM1638的STB，PK5连接TM1638的DIO，PM0连接TM1638的CLK）
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void GPIOInit(void)
{

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);		  // 使能端口 F
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}; // 等待端口 F准备完毕
		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);		   // 使能端口 J
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {}; // 等待端口 J准备完毕

  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);
		
	// 设置端口 J的第0位（PJ0）为输入引脚
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
	// 端口 J的第0位作为按键输入，类型设置成“推挽上拉”
  GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		
}

//*****************************************************************************
//
// 函数原型：SysTickInit(void)
// 函数功能：设置SysTick中断
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void SysTickInit(void)
{
  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms=1/50s
  SysTickEnable();  			// SysTick使能
  SysTickIntEnable();			// SysTick中断允许
}

//*****************************************************************************
//
// 函数原型：void DevicesInit(void)
// 函数功能：CU器件初始化，包括系统时钟设置、GPIO初始化和SysTick中断设置
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void DevicesInit(void)
{
  // 使用外部25MHz主时钟源，经过PLL，然后分频为20MHz
  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |
                                     SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480),
                                    20000000);

  GPIOInit();             // GPIO初始化
  SysTickInit();          // 设置SysTick中断
  IntMasterEnable();		// 总中断允许
}

//*****************************************************************************
//
// 函数原型：void SysTick_Handler(void)
// 函数功能：SysTick中断服务程序
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void SysTick_Handler(void)       // 定时周期为20ms,即每20ms自动进入一次服务函数
{
  // 0.1秒钟软定时器计数
  if (++clock100ms >= V_T100ms)
    {
      clock100ms_flag = 1;    // 当0.1秒到时，溢出标志置1
      clock100ms = 0;
    }

		
		current_PJ=GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0);
		
		if(previous_PJ==1 && current_PJ==0) switch_mode();
		previous_PJ=current_PJ;
  // 0.5秒钟软定时器计数
  /*if (++clock500ms >= V_T500ms)
    {
      clock500ms_flag = 1;    // 当0.5秒到时，溢出标志置1
      clock500ms = 0;
    }

  // 刷新全部数码管和LED指示灯
  TM1638_RefreshDIGIandLED(digit, pnt, led);

	show_key_number();//功能3常启
		*/
	
}



//****************************************************************************
//功能1
void FLASH_PF0(void)
{
	if (clock100ms_flag == 1)      // 检查0.1秒定时是否到
        {
          clock100ms_flag		= 0;
					if(light_flag==0)
					{
						light_flag=1;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);        // 关闭 LED4(D4-PF0)

					}
					else
					{
						light_flag=0;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);        // 关闭 LED4(D4-PF0)
					}
        }
}

//*************************************************************
void FLASH_PF4(void)
{
	if (clock100ms_flag == 1)      // 检查0.1秒定时是否到
        {
          clock100ms_flag		= 0;
					if(light_flag==0)
					{
						light_flag=1;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);        // 关闭 LED4(D4-PF0)

					}
					else
					{
						light_flag=0;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);        // 关闭 LED4(D4-PF0)
					}
        }
}

/*
//****************************************************
//功能2,从左向右
void flash_light_run_right(void)
{
	uint8_t temp,i;
	
	if (clock500ms_flag == 1)   // 检查0.5秒定时是否到
        {
          clock500ms_flag = 0;
          // 8个指示灯以走马灯方式，每0.5秒向右（循环）移动一格
          temp = led[0];
          for (i = 0; i < 7; i++) led[i] = led[i + 1];//每20ms都运行完整的for循环一次，for循环：走马灯
          led[7] = temp;
        }
}


//***********************************
//功能2，从右向左
void flash_light_run_left(void)
{
	uint8_t temp,i;
	
	if (clock500ms_flag == 1)   // 检查0.5秒定时是否到
		{
          clock500ms_flag = 0;
          // 8个指示灯以走马灯方式，每0.5秒向右（循环）移动一格
          temp = led[7];
          for (i = 7; i >0; i--) led[i] = led[i - 1];//每20ms都运行完整的for循环一次，for循环：走马灯
          led[0] = temp;
    }

}

		
//***********************************************************
//功能3
void show_key_number(void)
{
	
	// 检查当前键盘输入，0代表无键操作，1-9表示有对应按键
  // 键号显示在一位数码管上
  key_code = TM1638_Readkeyboard();

  digit[5] = key_code;//5号位线显示按键号
	
}
*/

//**********************************
//转变工作模式
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
//运行当前工作模式
void run_mode(void)
{
	
	if(mode==1) FLASH_PF0();
	if(mode==2) GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);        // 关闭 LED4(D4-PF0)
	if(mode==3) FLASH_PF4();
	if(mode==4) GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);        // 关闭 LED4(D4-PF0)


	
}

