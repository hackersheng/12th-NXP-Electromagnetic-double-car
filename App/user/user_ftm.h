/*
* @file       user_ftm.h
* @brief      ������������
* @author     Benson
*/

#ifndef __USER_FTM_H__
#define __USER_FTM_H__

void FTM_Init();
void motor(int32 motor_duty);
void get_speed();
void steer(int32 steer_duty);

#endif