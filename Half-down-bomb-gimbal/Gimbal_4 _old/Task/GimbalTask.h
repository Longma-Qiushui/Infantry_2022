#ifndef __GIMBALTASK_H
#define __GIMBALTASK_H

#include "main.h"
#include "stm32f4xx.h"
//#define gimbal_pitch_max 24
#define gimbal_pitch_mid -30
void Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv);
void Gimbal_DropShot_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv);
void Gimbal_Armor_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv);
void Gimbal_Buff_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv);
void Gimbal_SI_Cal(float Gimbal_yaw,float Gimbal_pitch);
void Gimbal_Powerdown_Cal(void);
void FuzzyMotorGimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv);
void FuzzyGyroGimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv);

void get_F(void);
void T_change(void);

	
void Gimbal_CurrentPid_Cal(void);

void PidGimbalMotor_Init(void);
void FuzzyPidGimbalMotor_Init(void);

void Gimbal_task(void *pvParameters);
#endif
