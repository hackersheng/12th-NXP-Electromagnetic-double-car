/*
* @file       user_control.h
* @brief      AD采集及分析
* @author     Benson
*/

#ifndef __USER_CONTROL_H__
#define __USER_CONTROL_H__

void AD_Init();
void get_voltage();
void err_analyze();
void road_analyze();
int16 set_speed_aim();
void speed_control();
void fuzzy_steer_control();
//void fuzzy_params_set();
int32 formula1(double x);
int32 formula2(double x);
int16 formula3(uint16 x, uint16 m);
int16 formula4(void);
int16 formula5(void);
int16 formula6(void);
#endif