#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include 用户自定义的头文件
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO口操作
#include  "MK60_uart.h"     //串口
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //低功耗定时器(延时)
#include  "MK60_i2c.h"      //I2C
#include  "MK60_spi.h"      //SPI
#include  "MK60_ftm.h"      //FTM
#include  "MK60_pit.h"      //PIT
#include  "MK60_rtc.h"      //RTC
#include  "MK60_adc.h"      //ADC
#include  "MK60_dac.h"      //DAC
#include  "MK60_dma.h"      //DMA
#include  "MK60_FLASH.h"    //FLASH
#include  "MK60_can.h"      //CAN
#include  "MK60_sdhc.h"     //SDHC

//#include  "MK60_usb.h"      //usb

/*第三方头文件*/
#include  "LQ_OLED.h"		//龙邱oled
#include  "freecars_uart.h"		//freecars上位机

/*自己写的头文件*/
#include "user_pit.h"
#include "user_pid.h"
#include "user_ftm.h"
#include "user_control.h"
#include "user_sys.h"
#include "fuzzy.h"
#include "flag.h"
#include "user_distance.h"
#include "user_ring.h"
#include "user_connection.h"
#include "user_oled.h"
#include "current.h"
#include "user_flash.h"
#endif  //__INCLUDE_H__
