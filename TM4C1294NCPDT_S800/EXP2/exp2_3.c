#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"

#include "driverlib/systick.h" // SysTick Driver 原型

//*****************************************************************************
//
// I2C GPIO chip address and resigster define
//
//*****************************************************************************
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

#define SYSTICK_FREQUENCY 50 // SysTick频率为50Hz，即循环定时周期20ms

#define V_T100ms 50 // 0.1s软件定时器溢出值，5个20ms

void Delay(uint32_t value);
void S800_GPIO_Init(void);
void SysTickInit(void);

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void S800_I2C0_Init(void);
void run(void);

volatile uint8_t result, result1;
uint32_t ui32SysClock;

uint8_t seg7[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x079, 0x71, 0x5c};
//uint8_t seg7_letter[]={};
uint8_t pnt=0xf0;

uint8_t clock1s = 0;
uint8_t clock1s_flag = 0;

uint8_t num = 1;
uint8_t pos = 1;

uint8_t which_light = 0;
uint8_t light[] = {0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};

uint8_t cur_PJ0 = 0;
uint8_t pre_PJ0 = 0;

int main(void)
{
    // use internal 16M oscillator, HSI
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ | SYSCTL_OSC_INT | SYSCTL_USE_OSC), 16000000);

    S800_GPIO_Init();
    S800_I2C0_Init();
    SysTickInit();

    while (1)
    {

        if (clock1s_flag == 1) // 检查0.1秒定时是否到
        {
            /*
            uint8_t index = 1;
            while (index <= 8)
            {
                run(index);
                ++index;
            }
            */

            run();
            clock1s_flag = 0; // 每0.1秒累加计时值在数码管上以十进制显示，有键按下时暂停计时
        }

        // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);												// Turn on the PF0
        // Delay(800000);
        // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);																// Turn off the PF0.
        // Delay(800000);
    }
}

void Delay(uint32_t value)
{
    uint32_t ui32Loop;
    for (ui32Loop = 0; ui32Loop < value; ui32Loop++)
    {
    };
}

void S800_GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable PortF
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
        ;                                        // Wait for the GPIO moduleF ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Enable PortJ
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
        ; // Wait for the GPIO moduleJ ready

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);             // Set PF0 as Output pin
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Set the PJ0,PJ1 as input pin

    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void)
{
    uint8_t result;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);     // 初始化i2c模块
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // 使用I2C模块0，引脚配置为I2C0SCL--PB2、I2C0SDA--PB3
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);             // 配置PB2为I2C0SCL
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);             // 配置PB3为I2C0SDA
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // I2C将GPIO_PIN_2用作SCL
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);    // I2C将GPIO_PIN_3用作SDA

    I2CMasterInitExpClk(I2C0_BASE, ui32SysClock, true); // config I2C0 400k
    I2CMasterEnable(I2C0_BASE);

    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT0, 0x0ff); // config port 0 as input
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT1, 0x0);   // config port 1 as output
    result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_CONFIG_PORT2, 0x0);   // config port 2 as output

    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_CONFIG, 0x00);  // config port as output
    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0x0ff); // turn off the LED1-8
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
    uint8_t rop;
    while (I2CMasterBusy(I2C0_BASE))
    {
    }; // 如果I2C0模块忙，等待
    //
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    // 设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机

    I2CMasterDataPut(I2C0_BASE, RegAddr);                         // 主机写设备寄存器地址
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

/*
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t value, rop;
    while (I2CMasterBusy(I2C0_BASE))
    {
    };
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    I2CMasterDataPut(I2C0_BASE, RegAddr);
    //	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND); // 执行单词写入操作
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    rop = (uint8_t)I2CMasterErr(I2C0_BASE);
    Delay(1);
    // receive data
    I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);            // 设置从机地址
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // 执行单次读操作
    while (I2CMasterBusBusy(I2C0_BASE))
        ;
    value = I2CMasterDataGet(I2C0_BASE); // 获取读取的数据
    Delay(1);
    return value;
    */

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

void SysTickInit(void)
{
    SysTickPeriodSet(ui32SysClock / SYSTICK_FREQUENCY); // 设置心跳节拍,定时周期20ms=1/50s
    SysTickEnable();                                    // SysTick使能
    SysTickIntEnable();                                 // SysTick中断允许
}

void SysTick_Handler(void) // 定时周期为20ms,即每20ms自动进入一次服务函数
{
    // 按下按键时计时暂停
    cur_PJ0 = GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0);
    if (cur_PJ0 == 0)
    {
    }
    else
    {
        ++clock1s;
    }

    // 0.1秒钟软定时器计数
    if (clock1s >= V_T100ms)
    {
        clock1s_flag = 1; // 当0.1秒到时，溢出标志置1
        clock1s = 0;
    }
}

void run(void)
{

    uint8_t i = 1;
    while (i < 100)
    {
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[num]);    // 段选，指定显示数字
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(2)); // 片选，选择特定位（对应为二进制序列）
        Delay(10000);

        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, 0);
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[num + 1]);
        result = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(4));
        Delay(10000);
        ++i;
    }

    // result1 = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT1, seg7[7]);    // 段选，指定显示数字
    // result1 = I2C0_WriteByte(TCA6424_I2CADDR, TCA6424_OUTPUT_PORT2, (uint8_t)(8)); // 片选，选择特定位（2号位）
    result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, light[which_light]); // 跑马灯：选择点亮的灯

    if (num == 8)
    {
        num = 1;
    }
    else
    {
        num++;
    }

    if (which_light == 7)
    {
        which_light = 0;
    }
    else
    {
        which_light++;
    }
}
