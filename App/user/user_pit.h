/*
* @file       user_pit.h
* @brief      包含各种与时间有关的标志位
* @author     Benson
*/

#ifndef __USER_PIT_H__
#define __USER_PIT_H__


typedef struct TIME{
  uint16 ss;
  uint16 ms;
}TIME;

TIME get_timevar(TIME time1,TIME time2);
uint16 get_timevar_ms(TIME time1,TIME time2);
void PIT_Init();

#endif