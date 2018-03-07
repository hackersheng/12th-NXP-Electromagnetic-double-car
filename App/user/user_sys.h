/*
* @file       user_sys.h
* @brief      ϵͳ
* @author     Benson
*/

#ifndef __USER_SYS_H__
#define __USER_SYS_H__

void SYS_Init();
void SYS_Pause();
void beep_init();
void beep_on();
void beep_off();
void user_key_init();
void buzzer(uint32 n);
void user_led_init(void);
void user_press_init(void);
void data_updata(void);
void startline_init();
uint8 get_startline();
#endif