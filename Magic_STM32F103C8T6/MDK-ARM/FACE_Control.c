//头文件调用
#include "main.h"
#include <Stdio.h>
#include <Stdbool.h>
#include "FACE_Control.h"
#include "hardware.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#define Face_Stop 0
#define Face_1 1
#define Face_2 2
#define Face_3 3
#define Face_4 4
#define Face_5 5
#define Face_6 6

#define MOTOR_ENABLE GPIO_PIN_RESET
#define MOTOR_DISABLE GPIO_PIN_SET

bool Mgk_PWM_Complete;

void Mgk_Ctrl_Face_Output(uint16_t face)
{
	switch(face)
	{
		case Face_Stop:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);

		break;		
		case Face_1:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_DISABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);
		break;		
			case Face_2:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_DISABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);
		break;	
		case Face_3:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_DISABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);
		break;	
		case Face_4:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_DISABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);
		break;	
		case Face_5:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_DISABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);
		break;	
		case Face_6:
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_DISABLE);
		break;			
		
		default: 
    HAL_GPIO_WritePin(Motor_EN_01_GPIO_Port,Motor_EN_01_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_02_GPIO_Port,Motor_EN_02_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_03_GPIO_Port,Motor_EN_03_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_04_GPIO_Port,Motor_EN_04_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_05_GPIO_Port,Motor_EN_05_Pin, MOTOR_ENABLE);
		HAL_GPIO_WritePin(Motor_EN_06_GPIO_Port,Motor_EN_06_Pin, MOTOR_ENABLE);
		break;	
		
	}    
}

void Mgk_Ctrl_Fulse_Output(bool out)
{
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, out);
}

void Mgk_Ctrl_Face_Dir(bool dir)
{

		HAL_GPIO_WritePin(Motor_DIR_GPIO_Port,Motor_DIR_Pin, dir);
}	


void delay_us(uint16_t i)               
{
	int j,k;
	for(j=0;j<71;j++)
		for(k=0;k<i;k++);
}
