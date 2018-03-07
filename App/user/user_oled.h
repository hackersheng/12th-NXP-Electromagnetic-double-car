#ifndef __OLED_H__
#define __OLED_H__

void display_get_max();
void user_oled_delay(int32 n);

void display_cal();
void display_cal_up();
void display_cal_down();


void display_ring_ctrl();

void display_pid();
void display_pid_up();
void display_pid_down();

void display_time();
void display_time_up();
void display_time_down();
void display_time_ok();


void display_speed_pid();

void display_ad();
void display_ad_up();
void display_ad_down();

void display_add();

void display_ring_ctrl_down();
void display_ring_ctrl_up();

void display_ring_cal(void);
void IrqHandler_page(void);

//void display_fuzzy_p();
//void display_fuzzy_p_up();
//void display_fuzzy_p_down();
//
//void display_fuzzy_d();
//void display_fuzzy_d_up();
//void display_fuzzy_d_down();

void display_ad_ok();

void user_page_init();
void create_page(void (*dis_fun)(void),void (*up_fun)(void),void (*down_fun)(void),void (*left_fun)(void),void (*right_fun)(void),void (*ok_fun)(void),int row,int page);
void create_disonly_page(void (*dis_fun)(void),int page);
void user_lcd_display();
void dis_logo();
#endif