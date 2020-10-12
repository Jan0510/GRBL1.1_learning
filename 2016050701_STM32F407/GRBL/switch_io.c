#include "grbl.h"

void SwitchIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTE时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;   //step引脚初始化
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                      //普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                     //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                  //50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);                                             //脉冲引脚与使能引脚初始化
}

void SetLPOSwitch(void)
{
	GPIO_SetBits(GPIOE,(1<<LPOSwitch_BIT));
}
void ResetLPOSwitch(void)
{
	GPIO_ResetBits(GPIOE,(1<<LPOSwitch_BIT));
}
void SetHPOSwitch(void)
{
	GPIO_SetBits(GPIOE,(1<<HPOSwitch_BIT));
}
void ResetHPOSwitch(void)
{
	GPIO_ResetBits(GPIOE,(1<<HPOSwitch_BIT));
}
void SetGasSwitch(void)
{
	GPIO_SetBits(GPIOE,(1<<GasSwitch_BIT));
}
void ResetGasSwitch(void)
{
	GPIO_ResetBits(GPIOE,(1<<GasSwitch_BIT));
}
void SetAutoSwitch(void)
{
	GPIO_SetBits(GPIOE,(1<<AutoSwitch_BIT));
}
void ResetAutoSwitch(void)
{
	GPIO_ResetBits(GPIOE,(1<<AutoSwitch_BIT));
}
