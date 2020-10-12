#include "grbl.h"

void SwitchIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTEʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;   //step���ų�ʼ��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                                      //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                                     //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                  //50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);                                             //����������ʹ�����ų�ʼ��
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
