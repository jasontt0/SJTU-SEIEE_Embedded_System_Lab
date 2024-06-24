/*
用SW1按键做按键切换，共8个模式，分别对应变量mode=0,1,2,...,8

mode=0对应黑屏，数码管不显示（自己加的），姓名学号闪烁结束后默认为此模式
mode=1：左慢速流水
mode=2：左快速流水
mode=3：右慢速流水
mode=4：右快速流水
mode=5：静态显示时间
mode=6：静态显示日期
mode=7：静态显示闹钟

*/

//*****************************************************************************
//
// 头文件
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"		 // 基址宏定义
#include "inc/hw_types.h"		 // 数据类型宏定义，寄存器访问函数
#include "inc/hw_ints.h"		 // 与中断有关的宏定义
#include "driverlib/debug.h"	 // 调试用
#include "driverlib/gpio.h"		 // 通用IO口宏定义和函数原型
#include "driverlib/pin_map.h"	 // TM4C系列MCU外围设备管脚宏定义
#include "driverlib/sysctl.h"	 // 系统控制定义
#include "driverlib/systick.h"	 // SysTick Driver 原型
#include "driverlib/interrupt.h" // NVIC Interrupt Controller Driver 原型
#include "driverlib/uart.h"		 // 与UART有关的宏定义和函数原型

// #include "tm1638.h" // 与控制TM1638芯片有关的宏定义和函数原型

#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"

#include "driverlib/systick.h" // SysTick Driver 原型

#include "driverlib/pwm.h"

#include "PWM.H"

//*****************************************************************************
//
// 宏定义
//
//*****************************************************************************
#define SYSTICK_FREQUENCY 50 // SysTick频率为50Hz，即循环定时周期20ms

#define V_T1s 50	// 1s软件定时器溢出值，50个20ms
#define V_T200ms 10 // 200ms软件定时器溢出值
#define V_T600ms 30 // 600ms软件定时器溢出值

#define TCA6424_I2CADDR 0x22
#define PCA9557_I2CADDR 0x18

#define PCA9557_INPUT 0x00
#define PCA9557_OUTPUT 0x01
#define PCA9557_POLINVERT 0x02
#define PCA9557_CONFIG 0x03

#define TCA6424_CONFIG_PORT0 0x0c
#define TCA6424_CONFIG_PORT1 0x0d
#define TCA6424_CONFIG_PORT2 0x0e

#define TCA6424_INPUT_PORT0 0x00
#define TCA6424_INPUT_PORT1 0x01
#define TCA6424_INPUT_PORT2 0x02

#define TCA6424_OUTPUT_PORT0 0x04
#define TCA6424_OUTPUT_PORT1 0x05
#define TCA6424_OUTPUT_PORT2 0x06

//*****************************************************************************
//
// 函数原型声明
//
//*****************************************************************************
void GPIOInit(void);	// GPIO初始化
void SysTickInit(void); // 设置SysTick中断

void UARTInit(void);												// UART初始化
void UARTStringPutNOBlocking(uint32_t ui32Base, uint8_t *cMessage); // 向UART发送字符串

void S800_I2C0_Init(void);
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);

void DevicesInit(void); // MCU器件初始化，注：会调用上述函数

void Delay(uint32_t value);

// 指令集处理函数-------------------------------------
bool is_INIT_CLOCK(uint8_t *str);

bool is_SET_TIME(uint8_t *str);
bool is_GET_TIME(uint8_t *str);

bool is_SET_DATE(uint8_t *str);
bool is_GET_DATE(uint8_t *str);

bool is_SET_ALARM(uint8_t *str);
bool is_GET_ALARM(uint8_t *str);
//
void run_INIT_CLOCK(void);

void run_SET_TIME(void);
void run_GET_TIME(void);

void run_SET_DATE(void);
void run_GET_DATE(void);

void run_SET_ALARM(void);
void run_GET_ALARM(void);
//------------------------------------------------

void func_set_time(uint8_t h, uint8_t m, uint8_t s);
void func_set_date(uint8_t y, uint8_t m, uint8_t d);
void func_set_alarm(uint8_t a_h, uint8_t a_m, uint8_t a_s);

void record_date_time_alarm_(void);

void show_time(void);
void show_date(void);
void show_alarm(void);

void left_shift_slow(void);
void right_shift_slow(void);
void left_shift_fast(void);
void right_shift_fast(void);


void switch_mode(void);
void run_mode(void);


//*****************************************************************************
//
// 变量定义
//
//*****************************************************************************

// 系统时钟频率
uint32_t ui32SysClock;

// 存储命令
uint8_t buffer[200] = {'\0'};
uint8_t uart_receive_len = 0;

// 命令是否接受完毕（出现\r置1）
uint8_t cmd_state = 0;

// 指令集--------------------------------
uint8_t INIT_CLOCK[] = "INITCLOCK";
uint8_t init_clock[] = "initclock";

uint8_t SET_TIME[] = "SETTIME"; // 输入格式：SET TIME 00:00:00，指令与时间之间有空格
uint8_t set_time[] = "settime";

uint8_t GET_TIME[] = "GETTIME";
uint8_t get_time[] = "gettime";

uint8_t SET_DATE[] = "SETDATE"; // 输入格式：SET DATE 00.00.00，指令与日期之间有空格
uint8_t set_date[] = "setdate"; //

uint8_t GET_DATE[] = "GETDATE";
uint8_t get_date[] = "getdate";

uint8_t SET_ALARM[] = "SETALARM"; // 输入格式：SET ALARM 00:00:00，指令与时间之间有空格
uint8_t set_alarm[] = "setalarm"; // 输入格式：SET ALARM 00:00:00，指令与时间之间有空格

uint8_t GET_ALARM[] = "GETALARM";
uint8_t get_alarm[] = "getalarm";
//-------------------------------------------

uint8_t hour = 0;
uint8_t minute = 0;
uint8_t second = 0;

uint8_t year = 0;
uint8_t month = 0;
uint8_t day = 0;

uint8_t alarm_hour = 99;
uint8_t alarm_minute = 59;
uint8_t alarm_second = 59;

uint8_t time_now[10] = "00:00:00";
uint8_t date_now[10] = "00.00.00";
uint8_t alarm_now[10] = "99:59:59";

bool T1s = 0;
bool T200ms = 0;
bool T600ms = 0;
uint8_t counter = 0;
uint8_t counter_200ms = 0;
uint8_t counter_600ms = 0;

uint32_t j = 0;

bool flag_times_up = 0;

bool flag_space = 0;
bool flag_command_have_space=0;

volatile uint8_t result;

uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c};
uint8_t seg7_name[] = {0x5e, 0x1c, 0x7e, 0x79, 0x54, 0x74, 0x77, 0x5c};//"D U W E N H A O"
uint8_t seg7_pnt = 0x80; // 小数点
uint8_t seg7_line = 0x40;
uint8_t seg7_space = 0x00;

uint8_t seg7_time_date_alarm[28];//存储数据用于显示

uint8_t pre_SW1 = 0xff;
uint8_t cur_SW1;

uint8_t mode = 0;

uint8_t left_riverfast_i = 0;
uint8_t left_riverslow_i = 0;

uint8_t right_riverfast_i = 135;
uint8_t right_riverslow_i = 135;

//*****************************************************************************
//
// 主程序
//
//*****************************************************************************
int main(void)
{

	DevicesInit(); //  MCU器件初始化

	SysCtlDelay(60 * (ui32SysClock / 3000)); // 延时>60ms,等待上电完成

	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"RESET\r\n");

	while (1)
	{
		record_date_time_alarm_();

		run_mode();

		if (T1s == 1)
		{
			T1s = 0;
			second += 1;

			if (flag_times_up == 0)
			{
				if (alarm_second != 0)
				{
					alarm_second -= 1;
				}
				else
				{
					alarm_second = 59;
					if (alarm_minute != 0)
					{
						alarm_minute -= 1;
					}
					else
					{

						if (alarm_hour != 0)
						{
							alarm_minute = 59;
							alarm_hour -= 1;
						}
						else
						{
							flag_times_up = 1;
						}
					}
				}
			}
		}

		if (T200ms == 1)
		{
			T200ms = 0;

			left_riverfast_i++;
			if (left_riverfast_i == 135)
			{
				left_riverfast_i = 0;
			}

			right_riverfast_i -= 1;
			if (right_riverfast_i == 0)
			{
				right_riverfast_i = 135;
			}
		}

		if (T600ms == 1)
		{
			T600ms = 0;

			left_riverslow_i++;
			if (left_riverslow_i == 135)
			{
				left_riverslow_i = 0;
			}

			right_riverslow_i -= 1;
			if (right_riverslow_i == 0)
			{
				right_riverslow_i = 135;
			}
		}

		if (cmd_state == 1)
		{
			cmd_state = 0;
			uart_receive_len = 0;

			if (is_INIT_CLOCK(buffer) && flag_command_have_space)
			{
				run_INIT_CLOCK();
				flag_command_have_space = 0;
			}
			else if (is_SET_TIME(buffer) && flag_command_have_space)
			{
				run_SET_TIME();
				flag_command_have_space = 0;
			}
			else if (is_GET_TIME(buffer) && flag_command_have_space)
			{
				run_GET_TIME();
				flag_command_have_space = 0;
			}
			else if (is_SET_DATE(buffer) && flag_command_have_space)
			{
				run_SET_DATE();
				flag_command_have_space = 0;
			}
			else if (is_GET_DATE(buffer) && flag_command_have_space)
			{
				run_GET_DATE();
				flag_command_have_space = 0;
			}
			else if (is_SET_ALARM(buffer) && flag_command_have_space)
			{
				run_SET_ALARM();
				flag_command_have_space = 0;
			}
			else if (is_GET_ALARM(buffer) && flag_command_have_space)
			{
				run_GET_ALARM();
				flag_command_have_space = 0;
			}

			else
			{
				UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"Error Command!\r");
				flag_command_have_space=0;
			}

			UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"\n");

			for (j = 0; j < 200; ++j)
				buffer[j] = 0; // 清空buffer
		}
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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // 使能端口 F
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
	{
	}; // 等待端口 F准备完毕

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // 使能端口 J
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
	{
	}; // 等待端口 J准备完毕

	// 设置端口 F的第0、4位（PF0,PF4）为输出引脚
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
	// 设置端口 J的第0、1位（PJ0,PJ1）为输入引脚
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
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
	SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms
	SysTickEnable();									// SysTick使能
	SysTickIntEnable();									// SysTick中断允许
}

//*****************************************************************************
//
// 函数原型：void UARTStringPut(uint32_t ui32Base,const char *cMessage)
// 函数功能：向UART模块发送字符串
// 函数参数：ui32Base：UART模块
//          cMessage：待发送字符串
// 函数返回值：无
//
//*****************************************************************************
void UARTStringPutNOBlocking(uint32_t ui32Base, uint8_t *cMessage)
{
	bool TXFIFO_free = 0;

	while (*cMessage != '\0')
	{
		TXFIFO_free = UARTCharPutNonBlocking(UART0_BASE, *(cMessage));
		if (TXFIFO_free)
		{
			cMessage++;
		}
		TXFIFO_free = 0;
	}
}

//*****************************************************************************
//
// 函数原型：void UARTInit(void)
// 函数功能：UART初始化。使能UART0，设置PA0,PA1为UART0 RX,TX引脚；
//          设置波特率及帧格式。
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void UARTInit(void)
{
	// 引脚配置
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // 使能UART0模块
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // 使能端口 A
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
		; // 等待端口 A准备完毕

	GPIOPinConfigure(GPIO_PA0_U0RX); // 设置PA0为UART0 RX引脚
	GPIOPinConfigure(GPIO_PA1_U0TX); // 设置PA1为UART0 TX引脚

	// 设置端口 A的第0,1位（PA0,PA1）为UART引脚
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// 波特率及帧格式设置
	UARTConfigSetExpClk(UART0_BASE,
						ui32SysClock,
						115200,					 // 波特率：115200
						(UART_CONFIG_WLEN_8 |	 // 数据位：8
						 UART_CONFIG_STOP_ONE |	 // 停止位：1
						 UART_CONFIG_PAR_NONE)); // 校验位：无

	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX7_8);
	IntEnable(INT_UART0);								  // UART0 中断允许
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); // 使能UART0 RX,RT 中断
}

//*******************************************************************
void S800_I2C0_Init(void)
{
	uint8_t result;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);		// 初始化i2c模块
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// 使用I2C模块0，引脚配置为I2C0SCL--PB2、I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);				// 配置PB2为I2C0SCL
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);				// 配置PB3为I2C0SDA
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // I2C将GPIO_PIN_2用作SCL
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);	// I2C将GPIO_PIN_3用作SDA

	I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); // config I2C0 400k
	I2CMasterEnable(I2C0_BASE);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0x0ff); // config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x0);   // config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x0);   // config port 2 as output

	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00);	 // config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); // turn off the LED1-8
}

//*****************************************************************************
//
// 函数原型：DevicesInit(void)
// 函数功能：CU器件初始化，包括系统时钟设置、GPIO初始化和SysTick中断设置
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void DevicesInit(void)
{
	// 使用外部25MHz主时钟源，经过PLL，然后分频为20MHz
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
									   SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
									  20000000);

	GPIOInit();	   // GPIO初始化
	SysTickInit(); // 设置SysTick中断
	UARTInit();	   // UART初始化
	S800_I2C0_Init();
	IntMasterEnable(); // 总中断允许

	PWMInit();
}

//**************************************************************************
uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while (I2CMasterBusy(I2C0_BASE))
	{
	}; // 如果I2C0模块忙，等待
	//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	// 设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机

	I2CMasterDataPut(I2C0_BASE, RegAddr);						  // 主机写设备寄存器地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // 执行重复写入操作
	while (I2CMasterBusy(I2C0_BASE))
	{
	};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE); // 调试用

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // 执行重复写入操作并结束
	while (I2CMasterBusy(I2C0_BASE))
	{
	};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE); // 调试用

	return rop; // 返回错误类型，无错返回0
}

//**********************************************************
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value;
	while (I2CMasterBusy(I2C0_BASE))
	{
	};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while (I2CMasterBusBusy(I2C0_BASE))
		;
	I2CMasterErr(I2C0_BASE);
	Delay(100);
	// receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while (I2CMasterBusBusy(I2C0_BASE))
		;
	Delay(100);
	value = I2CMasterDataGet(I2C0_BASE);
	Delay(100);
	return value;
}

//**********************************************
void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for (ui32Loop = 0; ui32Loop < value; ui32Loop++)
	{
	};
}

//*****************************************************************************
//
// 函数原型：void SysTick_Handler(void)
// 函数功能：SysTick中断服务程序
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void SysTick_Handler(void) // 定时周期为20ms
{
	if (counter < V_T1s)
	{
		++counter;
	}
	else
	{
		counter = 0;
		T1s = 1;
	}

	if (counter_200ms < V_T200ms)
	{
		++counter_200ms;
	}
	else
	{
		counter_200ms = 0;
		T200ms = 1;
	}

	if (counter_600ms < V_T600ms)
	{
		++counter_600ms;
	}
	else
	{
		counter_600ms = 0;
		T600ms = 1;
	}

	//
	if (second >= 60)
	{
		second -= 60;
		minute++;
	}
	if (minute >= 60)
	{
		minute -= 60;
		hour++;
	}
	if (hour >= 24)
	{
		hour -= 24;
		day++;
	}
	if (day > 30)
	{
		day -= 30;
		month++;
	}
	if (month > 12)
	{
		month -= 12;
		year++;
	}
	if (year >= 100)
	{
		year -= 100;
	}

	if (alarm_hour == 0 && alarm_minute == 0 && alarm_second == 0)
	{
		flag_times_up = 1;
	}
	else
	{
		flag_times_up = 0;
	}

	if (flag_times_up)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
		PWMStart(523);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
		PWMStop();
	}

	cur_SW1 = I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	if (cur_SW1 == 0xfe && pre_SW1 == 0xff)
	{
		switch_mode();
	}
	pre_SW1 = cur_SW1;
}

//*****************************************************************************
//
// 函数原型：void UART0_Handler(void)
// 函数功能：UART0中断服务程序
// 函数参数：无
// 函数返回值：无
//
//*****************************************************************************
void UART0_Handler(void)
{
	int32_t uart0_int_status;
	uint8_t uart_receive_char;
	volatile uint32_t i;

	uart0_int_status = UARTIntStatus(UART0_BASE, true); // 取中断状态
	UARTIntClear(UART0_BASE, uart0_int_status);			// 清中断标志

	while (UARTCharsAvail(UART0_BASE)) // 重复从接收FIFO 读取字符
	{
		uart_receive_char = UARTCharGetNonBlocking(UART0_BASE); // 读入一个字符

		if (uart_receive_char != '\r')
		{ // 命令未结束
			if (uart_receive_char == (uint8_t)' ')
			{
				flag_space = 1;
				flag_command_have_space = 1;
			}
			else
			{
				flag_space = 0;
			}

			if (flag_space == 0)
			{
				buffer[uart_receive_len] = uart_receive_char;
				++uart_receive_len;
			}

		}
		else
		{
			cmd_state = 1;
			buffer[uart_receive_len] = '\0';
		}
	}
}

// 设置时间（设置保存时间的变量）************************************************
void func_set_time(uint8_t h, uint8_t m, uint8_t s)
{
	second = s;
	minute = m;
	hour = h;
}

// 设置日期（设置保存日期的变量）************************************************
void func_set_date(uint8_t y, uint8_t m, uint8_t d)
{
	year = y;
	month = m;
	day = d;
}

// 设置闹钟（设置保存闹钟的变量）************************************************
void func_set_alarm(uint8_t a_h, uint8_t a_m, uint8_t a_s)
{
	alarm_hour = a_h;
	alarm_minute = a_m;
	alarm_second = a_s;
}

// 记录当前时间、日期、闹钟数据用于输出在串口通讯软件**********************************
void record_date_time_alarm_(void)
{

	if (second >= 60)
	{
		second -= 60;
		minute++;
	}
	if (minute >= 60)
	{
		minute -= 60;
		hour++;
	}
	if (hour >= 24)
	{
		hour -= 24;
		day++;
	}
	if (day > 30)
	{
		day -= 30;
		month++;
	}
	if (month > 12)
	{
		month -= 12;
		year++;
	}
	if (year >= 100)
	{
		year -= 100;
	}

	if (alarm_hour == 0 && alarm_minute == 0 && alarm_second == 0)
	{
		flag_times_up = 1;
	}
	else
	{
		flag_times_up = 0;
	}

	time_now[0] = hour / 10 + '0';
	time_now[1] = hour % 10 + '0';
	time_now[3] = minute / 10 + '0';
	time_now[4] = minute % 10 + '0';
	time_now[6] = second / 10 + '0';
	time_now[7] = second % 10 + '0';

	date_now[0] = year / 10 + '0';
	date_now[1] = year % 10 + '0';
	date_now[3] = month / 10 + '0';
	date_now[4] = month % 10 + '0';
	date_now[6] = day / 10 + '0';
	date_now[7] = day % 10 + '0';

	alarm_now[0] = alarm_hour / 10 + '0';
	alarm_now[1] = alarm_hour % 10 + '0';
	alarm_now[3] = alarm_minute / 10 + '0';
	alarm_now[4] = alarm_minute % 10 + '0';
	alarm_now[6] = alarm_second / 10 + '0';
	alarm_now[7] = alarm_second % 10 + '0';
}

// 指令集处理函数******************************************************
// 判断是否是INIT CLOCK
bool is_INIT_CLOCK(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 8; ++i)
	{
		if (str[i] != INIT_CLOCK[i] && str[i] != init_clock[i])
			return false;
	}

	if (str[10] == '\0')
		return true;
	return false;
}

// 判断SET TIME
bool is_SET_TIME(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 6; ++i)
	{
		if (str[i] != SET_TIME[i] && str[i] != set_time[i])
			return false;
	}

	if (str[9] == ':' && str[12] == ':' && str[16] == '\0')
	{
		return true;
	}

	return false;
}

// 判断GET TIME
bool is_GET_TIME(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 6; ++i)
	{
		if (str[i] != GET_TIME[i] && str[i] != get_time[i])
			return false;
	}

	if (str[8] == '\0')
		return true;
	return false;
}

// 判断SET DATE
bool is_SET_DATE(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 6; ++i)
	{
		if (str[i] != SET_DATE[i] && str[i] != set_date[i])
			return false;
	}

	if (str[9] == '.' && str[12] == '.' && str[16] == '\0')
	{
		return true;
	}

	return false;
}

// 判断GET DATE
bool is_GET_DATE(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 6; ++i)
	{
		if (str[i] != GET_DATE[i] && str[i] != get_date[i])
			return false;
	}

	if (str[8] == '\0')
		return true;
	return false;
}

// 判断SET ALARM
bool is_SET_ALARM(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 7; ++i)
	{
		if (str[i] != SET_ALARM[i] && str[i] != set_alarm[i])
			return false;
	}

	if (str[10] == ':' && str[13] == ':' && str[17] == '\0')
	{
		return true;
	}

	return false;
}

// 判断GET ALARM
bool is_GET_ALARM(uint8_t *str)
{
	uint8_t i = 0;
	for (; i <= 7; ++i)
	{
		if (str[i] != GET_ALARM[i] && str[i] != get_alarm[i])
			return false;
	}

	if (str[9] == '\0')
		return true;
	return false;
}

//-----------------------------
//
void run_INIT_CLOCK(void)
{
	func_set_date(0, 0, 0);
	func_set_time(0, 0, 0);
	func_set_alarm(99, 59, 59);

	record_date_time_alarm_();

	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"date_");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)date_now);
	UARTStringPutNOBlocking(UART0_BASE, " ");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"time_");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)time_now);
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"\n");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"alarm_");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)alarm_now);
}

// SET TIME 00:00:00
void run_SET_TIME(void)
{
	uint8_t h = (buffer[7] - '0') * 10 + (buffer[8] - '0');
	uint8_t m = (buffer[10] - '0') * 10 + (buffer[11] - '0');
	uint8_t s = (buffer[13] - '0') * 10 + (buffer[14] - '0');
	if (h > 23 || m > 59 || s > 59)
	{
		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"Error Command!\r");
	}
	else
	{
		func_set_time(h, m, s);

		record_date_time_alarm_();

		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"time_");
		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)time_now);
	}
}

//
void run_GET_TIME(void)
{
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"time_");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)time_now);
}

// SET DATE 00.00.00
void run_SET_DATE(void)
{
	uint8_t y = (buffer[7] - '0') * 10 + (buffer[8] - '0');
	uint8_t m = (buffer[10] - '0') * 10 + (buffer[11] - '0');
	uint8_t d = (buffer[13] - '0') * 10 + (buffer[14] - '0');
	if (y > 99 || m > 12 || d > 30)
	{
		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"Error Command!\r");
	}
	else
	{
		func_set_date(y, m, d);

		record_date_time_alarm_();

		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"date_");
		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)date_now);
	}
}

//
void run_GET_DATE(void)
{
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"date_");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)date_now);
}

// SET ALARM 00:00:00
void run_SET_ALARM(void)
{
	uint8_t a_h = (buffer[8] - '0') * 10 + (buffer[9] - '0');
	uint8_t a_m = (buffer[11] - '0') * 10 + (buffer[12] - '0');
	uint8_t a_s = (buffer[14] - '0') * 10 + (buffer[15] - '0');
	if (a_h > 99 || a_m > 59 || a_s > 59)
	{
		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"Error Command!\r");
	}
	else
	{
		func_set_alarm(a_h, a_m, a_s);

		record_date_time_alarm_();

		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"alarm_");
		UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)alarm_now);
	}
}

//
void run_GET_ALARM(void)
{
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)"alarm_");
	UARTStringPutNOBlocking(UART0_BASE, (uint8_t *)alarm_now);
}
//****************************************************************

//静态显示函数******************************************************
void show_time(void)
{
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[hour / 10]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);			 // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[hour % 10]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);			 // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[minute / 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[minute % 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[second / 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[second % 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);
	Delay(10000);
}

void show_date(void)
{
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[year / 10]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);			 // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[year % 10]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);			 // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_pnt); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x04);	  // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[month / 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[month % 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_pnt); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x20);	  // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[day / 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[day % 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);
	Delay(10000);
}

void show_alarm(void)
{
	// uint8_t i = 0;

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarm_hour / 10]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);				   // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarm_hour % 10]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);				   // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_line); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x04);	   // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarm_minute / 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarm_minute % 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_line); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x20);	   // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarm_second / 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[alarm_second % 10]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);
	Delay(10000);
}


//流水显示函数*****************************************************************
void left_shift_slow(void)
{
	seg7_time_date_alarm[0] = seg7[hour / 10];
	seg7_time_date_alarm[1] = seg7[hour % 10];
	seg7_time_date_alarm[2] = seg7_space;
	seg7_time_date_alarm[3] = seg7[minute / 10];
	seg7_time_date_alarm[4] = seg7[minute % 10];
	seg7_time_date_alarm[5] = seg7_space;
	seg7_time_date_alarm[6] = seg7[second / 10];
	seg7_time_date_alarm[7] = seg7[second % 10];
	seg7_time_date_alarm[8] = seg7_space;
	seg7_time_date_alarm[9] = seg7[year / 10];
	seg7_time_date_alarm[10] = seg7[year % 10];
	seg7_time_date_alarm[11] = seg7_pnt;
	seg7_time_date_alarm[12] = seg7[month / 10];
	seg7_time_date_alarm[13] = seg7[month % 10];
	seg7_time_date_alarm[14] = seg7_pnt;
	seg7_time_date_alarm[15] = seg7[day / 10];
	seg7_time_date_alarm[16] = seg7[day % 10];
	seg7_time_date_alarm[17] = seg7_space;
	seg7_time_date_alarm[18] = seg7[alarm_hour / 10];
	seg7_time_date_alarm[19] = seg7[alarm_hour % 10];
	seg7_time_date_alarm[20] = seg7_line;
	seg7_time_date_alarm[21] = seg7[alarm_minute / 10];
	seg7_time_date_alarm[22] = seg7[alarm_minute % 10];
	seg7_time_date_alarm[23] = seg7_line;
	seg7_time_date_alarm[24] = seg7[alarm_second / 10];
	seg7_time_date_alarm[25] = seg7[alarm_second % 10];
	seg7_time_date_alarm[26] = seg7_space;

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[left_riverslow_i % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);									// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 1) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);										  // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 2) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x04);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 3) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 4) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 5) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x20);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 6) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);										  // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverslow_i + 7) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);										  // write port 2
	Delay(10000);
}

//
void right_shift_slow(void)
{
	seg7_time_date_alarm[0] = seg7[hour / 10];
	seg7_time_date_alarm[1] = seg7[hour % 10];
	seg7_time_date_alarm[2] = seg7_space;
	seg7_time_date_alarm[3] = seg7[minute / 10];
	seg7_time_date_alarm[4] = seg7[minute % 10];
	seg7_time_date_alarm[5] = seg7_space;
	seg7_time_date_alarm[6] = seg7[second / 10];
	seg7_time_date_alarm[7] = seg7[second % 10];
	seg7_time_date_alarm[8] = seg7_space;
	seg7_time_date_alarm[9] = seg7[year / 10];
	seg7_time_date_alarm[10] = seg7[year % 10];
	seg7_time_date_alarm[11] = seg7_pnt;
	seg7_time_date_alarm[12] = seg7[month / 10];
	seg7_time_date_alarm[13] = seg7[month % 10];
	seg7_time_date_alarm[14] = seg7_pnt;
	seg7_time_date_alarm[15] = seg7[day / 10];
	seg7_time_date_alarm[16] = seg7[day % 10];
	seg7_time_date_alarm[17] = seg7_space;
	seg7_time_date_alarm[18] = seg7[alarm_hour / 10];
	seg7_time_date_alarm[19] = seg7[alarm_hour % 10];
	seg7_time_date_alarm[20] = seg7_line;
	seg7_time_date_alarm[21] = seg7[alarm_minute / 10];
	seg7_time_date_alarm[22] = seg7[alarm_minute % 10];
	seg7_time_date_alarm[23] = seg7_line;
	seg7_time_date_alarm[24] = seg7[alarm_second / 10];
	seg7_time_date_alarm[25] = seg7[alarm_second % 10];
	seg7_time_date_alarm[26] = seg7_space;

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 7) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);												// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 6) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);												// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 5) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x20);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 4) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 3) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);											// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 2) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x04);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverslow_i + 1) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);												// write port 2
	Delay(10000);
}

//
void left_shift_fast(void)
{
	seg7_time_date_alarm[0] = seg7[hour / 10];
	seg7_time_date_alarm[1] = seg7[hour % 10];
	seg7_time_date_alarm[2] = seg7_space;
	seg7_time_date_alarm[3] = seg7[minute / 10];
	seg7_time_date_alarm[4] = seg7[minute % 10];
	seg7_time_date_alarm[5] = seg7_space;
	seg7_time_date_alarm[6] = seg7[second / 10];
	seg7_time_date_alarm[7] = seg7[second % 10];
	seg7_time_date_alarm[8] = seg7_space;
	seg7_time_date_alarm[9] = seg7[year / 10];
	seg7_time_date_alarm[10] = seg7[year % 10];
	seg7_time_date_alarm[11] = seg7_pnt;
	seg7_time_date_alarm[12] = seg7[month / 10];
	seg7_time_date_alarm[13] = seg7[month % 10];
	seg7_time_date_alarm[14] = seg7_pnt;
	seg7_time_date_alarm[15] = seg7[day / 10];
	seg7_time_date_alarm[16] = seg7[day % 10];
	seg7_time_date_alarm[17] = seg7_space;
	seg7_time_date_alarm[18] = seg7[alarm_hour / 10];
	seg7_time_date_alarm[19] = seg7[alarm_hour % 10];
	seg7_time_date_alarm[20] = seg7_line;
	seg7_time_date_alarm[21] = seg7[alarm_minute / 10];
	seg7_time_date_alarm[22] = seg7[alarm_minute % 10];
	seg7_time_date_alarm[23] = seg7_line;
	seg7_time_date_alarm[24] = seg7[alarm_second / 10];
	seg7_time_date_alarm[25] = seg7[alarm_second % 10];
	seg7_time_date_alarm[26] = seg7_space;

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[left_riverfast_i % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);									// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 1) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);										  // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 2) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x04);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 3) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 4) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 5) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x20);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 6) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);										  // write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(left_riverfast_i + 7) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);										  // write port 2
	Delay(10000);
}

//
void right_shift_fast(void)
{
	seg7_time_date_alarm[0] = seg7[hour / 10];
	seg7_time_date_alarm[1] = seg7[hour % 10];
	seg7_time_date_alarm[2] = seg7_space;
	seg7_time_date_alarm[3] = seg7[minute / 10];
	seg7_time_date_alarm[4] = seg7[minute % 10];
	seg7_time_date_alarm[5] = seg7_space;
	seg7_time_date_alarm[6] = seg7[second / 10];
	seg7_time_date_alarm[7] = seg7[second % 10];
	seg7_time_date_alarm[8] = seg7_space;
	seg7_time_date_alarm[9] = seg7[year / 10];
	seg7_time_date_alarm[10] = seg7[year % 10];
	seg7_time_date_alarm[11] = seg7_pnt;
	seg7_time_date_alarm[12] = seg7[month / 10];
	seg7_time_date_alarm[13] = seg7[month % 10];
	seg7_time_date_alarm[14] = seg7_pnt;
	seg7_time_date_alarm[15] = seg7[day / 10];
	seg7_time_date_alarm[16] = seg7[day % 10];
	seg7_time_date_alarm[17] = seg7_space;
	seg7_time_date_alarm[18] = seg7[alarm_hour / 10];
	seg7_time_date_alarm[19] = seg7[alarm_hour % 10];
	seg7_time_date_alarm[20] = seg7_line;
	seg7_time_date_alarm[21] = seg7[alarm_minute / 10];
	seg7_time_date_alarm[22] = seg7[alarm_minute % 10];
	seg7_time_date_alarm[23] = seg7_line;
	seg7_time_date_alarm[24] = seg7[alarm_second / 10];
	seg7_time_date_alarm[25] = seg7[alarm_second % 10];
	seg7_time_date_alarm[26] = seg7_space;

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 7) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x80);												// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 6) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x40);												// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 5) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x20);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 4) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x10);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 3) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x08);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x01);											// write port 2
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 2) % 27]);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x04);
	Delay(10000);

	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7_time_date_alarm[(right_riverfast_i + 1) % 27]); // write port 1
	result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0x02);												// write port 2
	Delay(10000);
}


//----------------------------------------------
void switch_mode(void)
{
	if (mode == 0)
	{
		mode = 1;
	}
	else if (mode == 1)
	{
		mode = 2;
	}
	else if (mode == 2)
	{
		mode = 3;
	}
	else if (mode == 3)
	{
		mode = 4;
	}
	else if (mode == 4)
	{
		mode = 5;
	}
	else if (mode == 5)
	{
		mode = 6;
	}
	else if (mode == 6)
	{
		mode = 7;
	}
	else
	{
		mode = 0;
	}
}

void run_mode(void)
{
	if (mode == 1)
	{
		left_shift_slow();
	}
	else if (mode == 2)
	{
		left_shift_fast();
	}
	else if (mode == 3)
	{
		right_shift_slow();
	}
	else if (mode == 4)
	{
		right_shift_fast();
	}
	else if (mode == 5)
	{
		show_time();
	}
	else if (mode == 6)
	{
		show_date();
	}
	else if (mode == 7)
	{
		show_alarm();
	}
	else
	{
		result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)255);
		result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, 0x00); // write port 1
	}
}
