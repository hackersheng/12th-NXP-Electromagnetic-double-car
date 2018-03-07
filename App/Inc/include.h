#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MK60_wdog.h"
#include  "MK60_gpio.h"     //IO�ڲ���
#include  "MK60_uart.h"     //����
#include  "MK60_SysTick.h"
#include  "MK60_lptmr.h"    //�͹��Ķ�ʱ��(��ʱ)
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

/*������ͷ�ļ�*/
#include  "LQ_OLED.h"		//����oled
#include  "freecars_uart.h"		//freecars��λ��

/*�Լ�д��ͷ�ļ�*/
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
