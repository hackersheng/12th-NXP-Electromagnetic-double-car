
/*
flash的写入与读取
*/

#include "include.h"

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)
#define DATA_NUM	26

//////////要写入的参数//////////

extern int16 Spd_Kp;
extern int16 Spd_Ki;
extern int16 Spd_Kd;
extern int32 steer_limit;
extern uint16 Hori_ADMiddle_max;
extern int16 Set_Speed_Temp;
extern uint16 ad_coeff;
extern int16 Cur_Kp;
extern int16 Cur_Ki;
extern int16 ring_th;
extern int16 ring_tl;
extern int16 ring_adh;
extern int16 ring_adl;
extern int16 ring_td;
extern int16 Set_Speed_list[8];
extern int8 MODE;
extern int8 front_or_after;
extern int8 last_front_or_after;
extern int8 big_ring_pos;
extern int8 shift_gear;
////////////////////////////////


int32 flash_data[DATA_NUM];

void user_flash_init()
{
	flash_init();
}

void user_flash_prms()
{
	int i;
	flash_data[0]=Spd_Kp;
	flash_data[1]=Spd_Ki;
	flash_data[2]=Spd_Kd;
	flash_data[3]=steer_limit;
        //flash_data[3]=300;
	flash_data[4]=Hori_ADMiddle_max;
	flash_data[5]=Set_Speed_Temp;
	flash_data[6]=ad_coeff;
	flash_data[7]=Cur_Kp;
	flash_data[8]=Cur_Ki;
	flash_data[9]=ring_th;
	flash_data[10]=ring_tl;
	flash_data[11]=ring_adh;
	flash_data[12]=ring_adl;
	flash_data[13]=ring_td;
	for (i = 0; i < 8; i++)
	{
		flash_data[i+14] = Set_Speed_list[i];
	}
        flash_data[22]=MODE;
        flash_data[23]=front_or_after;
        flash_data[24]=big_ring_pos;
        flash_data[25]=shift_gear;
}

void user_flash_write()
{
	int i;
	flash_erase_sector(SECTOR_NUM);
	user_flash_prms();
	for(i=0;i<DATA_NUM;i++)
	{
		if( 1 == flash_write(SECTOR_NUM, i*4 , flash_data[i]) )
			;
		else
		{
			beep_on();
			while(1)
			{
				printf("write flash err in %d",i);
			}
		}
	}

//	while(1);
}



void user_flash_read()
{
	int i;
	for(i=0;i<DATA_NUM;i++)
	{
		flash_data[i]=flash_read(SECTOR_NUM, i*4, int16);
	}
	Spd_Kp=flash_data[0];
	Spd_Ki=flash_data[1];
	Spd_Kd=flash_data[2];
	steer_limit=flash_data[3];
	Hori_ADMiddle_max=flash_data[4];
	Set_Speed_Temp=flash_data[5];
	ad_coeff=flash_data[6];
	Cur_Kp=flash_data[7];
	Cur_Ki=flash_data[8];
	ring_th=flash_data[9];
	ring_tl=flash_data[10];
	ring_adh=flash_data[11];
	ring_adl=flash_data[12];
	ring_td=flash_data[13];
	for (i = 0; i < 8; i++)
	{
		Set_Speed_list[i] = flash_data[i + 14];
	}
    MODE=flash_data[22];
    front_or_after=flash_data[23];
    if(front_or_after!=1 && front_or_after!=2)        
        front_or_after=1;
    if(front_or_after==1)
       last_front_or_after=2;
    else
       last_front_or_after=1;
    
    big_ring_pos=flash_data[24];
    shift_gear=flash_data[25];
}
