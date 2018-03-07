 #include "include.h"
#include "common.h"

extern uint8 Dis_Coeff;
extern int8 front_or_after;
extern int8 ad_check_flag;
extern uint8 Ring_Cnt;
extern int8 last_front_or_after;
extern uint16 H_left,H_right,H_middle,V_left,V_right,Z_middle;
extern uint16 H_1,H_2,H_3,H_4,H_5;
extern uint16 Hori_ADMiddle_max;
extern int8 Road_Type;
extern int16 Set_Speed;
int16 Set_Speed_Temp;
int8 shift_gear=0;
extern int16 Spd_Kp;
extern int16 Spd_Ki;
extern int16 Spd_Kd;
extern uint16 Distance[3];
//extern int8 Timer_AD_MAX;
extern int16 Dir_Kd,Dir_Kp;
//extern int32 Motor_Duty;
extern int16 Steer_Duty[4];
extern int16 Dir_Err[4];
extern int32 chabihe,daoshucha,gongshi;
extern int16 Spd_Err[3];
extern uint8 Receive_Data1[6];
//extern int8 Dis_Flag;
extern int8 Emergency_Stop;
extern int8 Stop_Flag;
extern TIME sys_time;
extern TIME time_launch;
extern uint8 Dis_Coeff;
extern byte Kairyu[8*128];
extern byte dis_ram[8*128];
extern int16 ring_angle[4];
extern uint8 Overtake_ctrl_table[];
extern uint16 ad_coeff;
extern uint16 Hori_ADMiddle_max;
extern int32 steer_limit;
extern int16 Cur_Kp;
extern int16 Cur_Ki;
extern int16 ring_td;
extern int16 ring_th;
extern int16 ring_tl;
extern int16 ring_adh;
extern int16 ring_adl;
extern uint16 H_1,H_2,H_3,H_4,H_5;
extern uint16 ring_ad_max;
//extern int16 weaken_coeff;
extern int16 Set_Speed_list[8];
extern int8 MODE;	//档位
extern int8 big_ring_pos;
extern int16 cons;
extern char reset_info[15];
extern TIME time_start;
//extern int32 V_chabihe;
//extern int16 fuzzy_p_params_list[7][5];
//extern int16 fuzzy_d_params_list[7][5];
extern uint16 front_max,back_min;
/**************************************/

typedef struct DISPLAY{
	void (*dis_fun)(void);	//显示
	void (*up_fun)(void);	//上
	void (*down_fun)(void);	//下
	void (*left_fun)(void);	//左
	void (*right_fun)(void);//右
	void (*ok_fun)(void);	//中
	int row_max;
	int page;
}DISPLAY;


#define PAGE		4	//总页数



int8 gui_page=2;	//页码
int8 gui_row=1;	//行数 一共9行0~8
int8 ok_flag=0;		//中键标志位

DISPLAY dis_stack[PAGE];


void user_oled_delay(int32 n)
{
	int x,y;
	for(x=0;x<n;x++)
		for(y=0;y<100;y++);
}


/*
默认按键功能
*/
void default_up_fun()
{
	gui_row--;
	if(gui_row<=0)
		gui_row=dis_stack[gui_page].row_max;
}

void default_down_fun()
{
	gui_row++;
	if(gui_row>dis_stack[gui_page].row_max)
		gui_row=1;
}
void default_left_fun()
{
	gui_row=1;
	ok_flag=0;
	gui_page--;
	if(gui_page<=0)
		//gui_page=PAGE-1;
        gui_page=1;
}
void default_right_fun()
{
	ok_flag=0;
	gui_row=1;
	gui_page++;
	if(gui_page>=PAGE)
		gui_page=PAGE-1;
}
void default_ok_fun()
{
	ok_flag=~ok_flag;
	//if(gui_page==0)
		//current_regulate();	//电流环的初始值校准
}

/*
创建一个按键带特殊功能的页面
*/
void create_page(void (*dis_fun)(void),void (*up_fun)(void),void (*down_fun)(void),void (*left_fun)(void),void (*right_fun)(void),void (*ok_fun)(void),int row,int page)
{
	dis_stack[page].dis_fun=dis_fun;
	dis_stack[page].up_fun=up_fun;
	dis_stack[page].down_fun=down_fun;
	dis_stack[page].left_fun=left_fun;
	dis_stack[page].right_fun=right_fun;
	dis_stack[page].ok_fun=ok_fun;
	dis_stack[page].page=page;
	dis_stack[page].row_max=row;
}
/*
创建一个仅用于显示的页面
*/
void create_disonly_page(void (*dis_fun)(void),int page)
{
	dis_stack[page].dis_fun=dis_fun;
	dis_stack[page].up_fun=default_up_fun;
	dis_stack[page].down_fun=default_down_fun;
	dis_stack[page].left_fun=default_left_fun;
	dis_stack[page].right_fun=default_right_fun;
	dis_stack[page].ok_fun=default_ok_fun;
	dis_stack[page].page=page;
	dis_stack[page].row_max=0;
}
/*
初始化
*/
void user_page_init()
{
	//create_disonly_page(dis_logo,0);
	create_page(display_ad,display_ad_up,display_ad_down,default_left_fun,default_right_fun,display_ad_ok,0,3);
	//create_page(display_pid,display_pid_up,display_pid_down,default_left_fun,default_right_fun,default_ok_fun,7,2);
	//create_disonly_page(display_speed_pid,3);     //距离
	create_page(display_ring_ctrl,display_ring_ctrl_up,display_ring_ctrl_down,default_left_fun,default_right_fun,display_ad_ok,0,1);       //环岛选择
	//create_page(display_cal,display_cal_up,display_cal_down,default_left_fun,default_right_fun,default_ok_fun,5,4);     //环岛条件
	//create_disonly_page(display_receive,6);
	//create_disonly_page(display_add,4);
	create_page(display_time, display_time_up, display_time_down, default_left_fun, default_right_fun, display_time_ok, 5, 2);
//	create_page(display_fuzzy_p,display_fuzzy_p_up, display_fuzzy_p_down, default_left_fun, default_right_fun, default_ok_fun,5,9);
//	create_page(display_fuzzy_d,display_fuzzy_d_up, display_fuzzy_d_down, default_left_fun, default_right_fun, default_ok_fun,5,10);
}

/*
显示
*/
void user_lcd_display()
{
	if((dis_stack+gui_page)->dis_fun== NULL)
		user_lcd_cls();
	else
		(dis_stack+gui_page)->dis_fun();
	
	lcd_refresh();
}


/*
按键中断服务函数
*/
void IrqHandler_page(void)
{
  if(PORTE_ISFR & (1<<6))
  {
	  user_oled_delay(250);
	  if(!gpio_get(PTE6))
	  {
		  while(!gpio_get(PTE6));
	  	(dis_stack+gui_page)->ok_fun();
	  }
      PORTE_ISFR=(1<<6);
  }
  else if(PORTE_ISFR & (1<<8))
  {
	  user_oled_delay(250);
	  if(!gpio_get(PTE8))
	  {
		  while(!gpio_get(PTE8));
		(dis_stack+gui_page)->down_fun();
	  }
    PORTE_ISFR=(1<<8);
  }
  else if(PORTE_ISFR & (1<<7))
  {
	  if(Set_Speed!=0)
		  Set_Speed=0;
	  user_oled_delay(250);
	  if(!gpio_get(PTE7))
	  {
		  while(!gpio_get(PTE7));
		(dis_stack+gui_page)->right_fun();
	  }
    PORTE_ISFR=(1<<7);
  }
  else if(PORTE_ISFR & (1<<4))
  {
	  user_oled_delay(250);
	  if(!gpio_get(PTE4))
	  {
		  while(!gpio_get(PTE4));
		(dis_stack+gui_page)->up_fun();
	  }
	PORTE_ISFR=(1<<4);
  }	
  else if(PORTE_ISFR & (1<<5))
  {
	  user_oled_delay(250);
	  if(!gpio_get(PTE5))
	  {
		  while(!gpio_get(PTE5));
		(dis_stack+gui_page)->left_fun();
	  }
    PORTE_ISFR=(1<<5);
  }	  
}


///////////////////////////////////////////////////////////
////////////////////////显示内容///////////////////////////
//////////////////////////////////////////////////////////

/*
	显示logo
	第0页
*/
void dis_logo()
{
	int i;
	for(i=0;i<8*128;i++)
		dis_ram[i]=Kairyu[i];
}

/*
	环岛判定参数
*/
void display_cal()
{
  uint16 ml,mr,msum;
  int i;
//  ml=V_left*1000/(H_left+1);
//  mr=V_right*1000/(H_right+1);
  	ml=H_middle-H_left;
	mr=H_middle-H_right;
  msum=ml+mr;
  user_lcd_cls();
  user_lcd_p6x8str(0,0,"l/m");
  user_lcd_p6x8num4(0,1,ml);
  user_lcd_p6x8str(36,0,"lr");
  user_lcd_p6x8num4(36,1,abs(H_left-H_right));
  user_lcd_p6x8str(72,0,"r/m");
  user_lcd_p6x8num4(72,1,mr);
//  user_lcd_p6x8str(0,0,"tl");
//  user_lcd_p6x8num4(0,1,back_min-front_max);
//  user_lcd_p6x8str(72,0,"td");
//  user_lcd_p6x8num4(72,1,H_right-H_left);
  
  user_lcd_p6x8str(18,3,"td");	//
  user_lcd_p6x8num4(80,3,ring_td);
  
  user_lcd_p6x8str(18,4,"th");	//
  user_lcd_p6x8num4(80,4,ring_th);
  
  user_lcd_p6x8str(18,5,"tl");	//
  user_lcd_p6x8num4(80,5,ring_tl);
  
  user_lcd_p6x8str(18,6,"adh");	//
  user_lcd_p6x8num4(80,6,ring_adh);
  
  user_lcd_p6x8str(18,7,"adl");	//
  user_lcd_p6x8num4(80,7,ring_adl);
//  user_lcd_p6x8str(0,4,"***");
//  user_lcd_p6x8str(72,4,"z_m");
//  user_lcd_p6x8num4(0,6,(V_left>V_right)?(V_left*1000/V_right):(V_right*1000/V_left));
//  user_lcd_p6x8num4(72,6,Z_middle);
 
  //显示选中符号
  if(ok_flag==0)
  {
	  for(i=0;i<dis_stack[gui_page].row_max;i++)
	  {
		  if(i+1==gui_row)
		 	user_lcd_p6x8str(0,i+3,"* ");
		  else
			user_lcd_p6x8str(0,i+3,"  ");
	  }
  }
  else
  {
	  for(i=0;i<dis_stack[gui_page].row_max;i++)
	  {
		  if(i+1==gui_row)
		 	user_lcd_p6x8str(0,i+3,"**");
		  else
			user_lcd_p6x8str(0,i+3,"  ");
	  }
  }
  
  lcd_refresh();
}
//此页的按键功能
void display_cal_up()
{
	if(ok_flag==0)
	{
		default_up_fun();
	}
	else
	{
		switch(gui_row)
		{
			case 1: ring_td+=10;	break;
			case 2: ring_th+=10;	break;
			case 3: ring_tl+=10;	break;
			case 4:	ring_adh+=10;	break;
			case 5:	ring_adl+=10;	break;
		}
	}
	
	/**避免flash为空，出现参数为-1的情况**/
	if(ring_td<10)	ring_td=200;
	if(ring_th<10)	ring_th=600;
	if(ring_tl<10)	ring_tl=500;
	if(ring_adh<10)	ring_adh=250;
	if(ring_adl<10)	ring_adl=50;
}
void display_cal_down()
{
	if(ok_flag==0)
	{
		default_down_fun();
	}
	else
	{
		switch(gui_row)
		{
			case 1: ring_td-=10;	break;
			case 2: ring_th-=10;	break;
			case 3: ring_tl-=10;	break;
			case 4:	ring_adh-=10;	break;
			case 5:	ring_adl-=10;	break;
		}
	}
}


/*
	显示参数
*/
void display_pid()
{
	int i;
	
  user_lcd_cls();
  user_lcd_p6x8str(18,1,"spd_p");
  user_lcd_p6x8num4(80,1,(int16)(Spd_Kp));
  
  user_lcd_p6x8str(18,2,"spd_i");
  user_lcd_p6x8num4(80,2,(int16)(Spd_Ki));
  
  user_lcd_p6x8str(18,3,"spd_d");
  user_lcd_p6x8num4(80,3,(int16)(Spd_Kd));
  
  user_lcd_p6x8str(18,4,"cur_p");
  user_lcd_p6x8num4(80,4,Cur_Kp);
  
  user_lcd_p6x8str(18,5,"cur_i");
  user_lcd_p6x8num4(80,5,Cur_Ki);
  
  user_lcd_p6x8str(18,6,"limit");	//7
  user_lcd_p6x8num4(80,6,(int16)steer_limit);
  
  user_lcd_p6x8str(18,7,"f_or_a");	//7
  user_lcd_p6x8num4(80,7,front_or_after);
  
  
  if(ok_flag==0)
  {
	  for(i=0;i<dis_stack[gui_page].row_max;i++)
	  {
		  if(i+1==gui_row)
		 	user_lcd_p6x8str(0,i+1,"* ");
		  else
			user_lcd_p6x8str(0,i+1,"  ");
	  }
  }
  else
  {
	  for(i=0;i<dis_stack[gui_page].row_max;i++)
	  {
		  if(i+1==gui_row)
		 	user_lcd_p6x8str(0,i+1,"**");
		  else
			user_lcd_p6x8str(0,i+1,"  ");
	  }
  }
  
  /*
  user_lcd_p6x8str(18,6,"");
  user_lcd_p6x8str(18,7,"spd_i");
  user_lcd_p6x8str(18,8,"spd_i");
  */
  
  lcd_refresh();
}
void display_pid_up()
{
	if(ok_flag==0)
	{
		default_up_fun();
	}
	else
	{
		switch(gui_row)
		{
			case 1: Spd_Kp+=10;	break;
			case 2: Spd_Ki+=1;	break;
			case 3: Spd_Kd+=100;	break;
			case 4: Cur_Kp+=100;	break;
			case 5: Cur_Ki+=100;	break;
			case 6: steer_limit++;	break;
			case 7: front_or_after=(front_or_after==1)?2:1;
					last_front_or_after=(front_or_after==1)?2:1;
			break;
		}
	}
	
	/**避免flash为空，出现参数为-1的情况**/
	if(Spd_Kp<10)	Spd_Kp=3000;
	if(Spd_Ki<10)	Spd_Ki=5000;
	if(Spd_Kd<10)	Spd_Kd=500;
	if(Cur_Kp<10)	Cur_Kp=500;
	if(Cur_Ki<10)	Cur_Ki=2200;
	if(steer_limit<10)	steer_limit=300;
	if(cons<10)	Hori_ADMiddle_max=720;
}
void display_pid_down()
{
	if(ok_flag==0)
	{
		default_down_fun();
	}
	else
	{
		switch(gui_row)
		{
			case 1: Spd_Kp-=10;	break;
			case 2: Spd_Ki-=1;	break;
			case 3: Spd_Kd-=100;	break;
			case 4: Cur_Kp-=100;	break;
			case 5: Cur_Ki-=100;	break;
			case 6: steer_limit--;	break;
			case 7: front_or_after=(front_or_after==1)?2:1;
					last_front_or_after=(front_or_after==1)?2:1;
			break;
		}
	}
}

/*
	发车页
*/
void display_time()
{
	int i;
   	user_lcd_cls();
// 	user_lcd_p6x8str(0,0,"time");
//	user_lcd_p6x8num4(0,2, sys_time.ss);
//	user_lcd_p6x8num4(60,2, sys_time.ms);
    user_lcd_p6x8str(0,0,"dis");
    user_lcd_p6x8num4(20,0, Distance[2]);
    user_lcd_p6x8str(64,1,"type");
    if(Road_Type<0)
    {
      user_lcd_p6x8str(90+24-6,1,"-_");
      user_lcd_p6x8num_single(90+24,1, -Road_Type);
    }
    else
      user_lcd_p6x8num_single(90+24,1, Road_Type);
    user_lcd_p6x8str(0,1,"ad_");
    user_lcd_p6x8num_single(20+24,1, ad_check_flag);
    user_lcd_p6x8str(64,0,"err");
    user_lcd_p6x8num3(96,0, Dir_Err[2]);
    
    user_lcd_p6x8str(16, 3, "start!!");
	user_lcd_p6x8str(16,4,"speed");
	user_lcd_p6x8num3(66+6,4, (int16)(Set_Speed_Temp));
	user_lcd_p6x8str(16, 5, "mode");
	user_lcd_p6x8num_single(66+24, 5, MODE);
    user_lcd_p6x8str(16, 6, "shift");
	user_lcd_p6x8num_single(66+24, 6, shift_gear);
	user_lcd_p6x8str(16, 7, "rank");
    user_lcd_p6x8num_single(66+24,7, front_or_after);

	if (ok_flag == 0)
	{
		for (i = 0; i<dis_stack[gui_page].row_max; i++)
		{
			if (i + 1 == gui_row)
				user_lcd_p6x8str(0, i + 3, "* ");
			else
				user_lcd_p6x8str(0, i + 3, "  ");
		}
	}
	else
	{
		for (i = 0; i<dis_stack[gui_page].row_max; i++)
		{
			if (i + 1 == gui_row)
				user_lcd_p6x8str(0, i + 3, "**");
			else
				user_lcd_p6x8str(0, i + 3, "  ");
		}
	}


	lcd_refresh();
}
void display_time_up()
{
	if (ok_flag == 0)
	{
		default_up_fun();
	}
	else 
	{
		switch (gui_row)
		{
		case 2:
			Set_Speed_list[MODE] = Set_Speed_list[MODE] + 1;
			Set_Speed_Temp = Set_Speed_list[MODE];
			break;
		case 3:MODE++;
			if (MODE > 7)	MODE = 0;
            Set_Speed_Temp = Set_Speed_list[MODE];
			break;
        case 4:
            shift_gear++;
            if(shift_gear>3)    shift_gear=0;
            break;
        case 5:
            front_or_after=(front_or_after==1)?2:1;
            last_front_or_after=(front_or_after==1)?2:1;            
            break;
		}
	}
	
	if (Set_Speed_list[MODE] < 1)	Set_Speed_list[MODE] = 80;
	if (MODE < 0)				MODE = 0;
}

void display_time_down()
{
	if (ok_flag == 0)
	{
		default_down_fun();
	}
	else
	{
		switch (gui_row)
		{
		case 2:
			Set_Speed_list[MODE] = Set_Speed_list[MODE] - 1;
			Set_Speed_Temp = Set_Speed_list[MODE];
			break;
		case 3:MODE--;
			if (MODE < 0)	MODE = 0;
			Set_Speed_Temp = Set_Speed_list[MODE];
			break;
        case 4:
            shift_gear--;
            if(shift_gear<0)    shift_gear=3;
            break;
        case 5:
            front_or_after=(front_or_after==1)?2:1;
            last_front_or_after=(front_or_after==1)?2:1;                        
            break;
		}
	}
}
void display_time_ok()
{
	if (Set_Speed == 0)
	{
		if (gui_row == 1)
		{
			user_flash_write();
			user_oled_delay(200000);
			Emergency_Stop = 0; Stop_Flag = 0; Set_Speed = Set_Speed_Temp; time_launch = sys_time; time_start=sys_time;
		}
		else
		{
			default_ok_fun();
		}
	}
	else
	{
		Stop_Flag=1;
		Set_Speed=0;
	}
}


/*
	距离
*/
void display_speed_pid()
{
  user_lcd_cls();
//  user_lcd_p6x8str(6,0,"v_ch");//
//  user_lcd_p6x8num4(60, 0, V_chabihe);
  user_lcd_p6x8str(6,2,"dis_coe");//前后车的距离    
  user_lcd_p6x8num4(60, 2, Dis_Coeff);
  user_lcd_p6x8str(8,4,"dis");//前后车的距离    
  user_lcd_p6x8num4(60, 4, Distance[2]);
  user_lcd_p6x8num4(0, 6, front_or_after);
  user_lcd_p6x8num4(60, 6, last_front_or_after);
  lcd_refresh();
}

/*
	环岛控制
*/
void display_ring_ctrl()
{
	user_lcd_cls();
	user_lcd_p6x8str(0,0,"ring1");
	user_lcd_p6x8str(0,1,"ring2");
	user_lcd_p6x8str(0,2,"ring3");
	user_lcd_p6x8str(0,3,"ring4");
	user_lcd_p6x8str(48,0, (Overtake_ctrl_table[0])?"yes":"no ");
	user_lcd_p6x8str(48,1, (Overtake_ctrl_table[1])?"yes":"no ");
	user_lcd_p6x8str(48,2, (Overtake_ctrl_table[2])?"yes":"no ");
	user_lcd_p6x8str(48,3, (Overtake_ctrl_table[3])?"yes":"no ");
	user_lcd_p6x8str(72,0, (ring_angle[0]>0)?"right":"left ");
	user_lcd_p6x8str(72,1, (ring_angle[1]>0)?"right":"left ");
	user_lcd_p6x8str(72,2, (ring_angle[2]>0)?"right":"left ");
	user_lcd_p6x8str(72,3, (ring_angle[3]>0)?"right":"left ");
    
    user_lcd_p6x8str(0,4, "ring_count");
    user_lcd_p6x8num_single(80,4,Ring_Cnt);

    user_lcd_p6x8str(0,5, "big ring at");
    user_lcd_p6x8num_single(80,5,big_ring_pos);
    
}

void display_ring_ctrl_up()
{
    if(big_ring_pos<=0)
        big_ring_pos=2;
    big_ring_pos++;
    if(big_ring_pos>Ring_Cnt_max)
        big_ring_pos=1;
}

void display_ring_ctrl_down()
{
    if(big_ring_pos<=0)
        big_ring_pos=2;
    big_ring_pos--;
    if(big_ring_pos<=0)
        big_ring_pos=Ring_Cnt_max;
}
/*
	首页
*/
void display_ad()
{
  user_lcd_cls();
  
  user_lcd_p6x8num3(0,0,sys_time.ss);
  user_lcd_p6x8str(40,0,reset_info);
  
  user_lcd_p6x8num3(0, 2, V_left);
  //user_lcd_p6x8str(62,0,"|");
  user_lcd_p6x8num3(98, 2, V_right);
  
  user_lcd_p6x8num3(0, 3, H_left);
  user_lcd_p6x8num3(53,3,H_middle);
  user_lcd_p6x8num3(98, 3, H_right);
  
  user_lcd_p6x8num3(0, 4, H_1);
  user_lcd_p6x8num3(53,4, H_3);
  user_lcd_p6x8num3(98,4, H_5);
  
//  user_lcd_p6x8num_single(110, 4, (Road_Type>0)?Road_Type:(-Road_Type));
//  
//  if(Road_Type>0)
//    user_lcd_p6x8str(100,4,"+");
//  else if(Road_Type<0)
//    user_lcd_p6x8str(100,4,"-");
//  else
//    user_lcd_p6x8str(100,4," ");
  user_lcd_p6x8str(6,6,"coeff");
  user_lcd_p6x8num3(60,6, ad_coeff);
  
  
  
//  user_lcd_p6x8str(0,6,"err");
//  user_lcd_p6x8num3(60, 6, Dir_Err[3]);
//  user_lcd_p6x8str(0, 7, "steer");
//  user_lcd_p6x8num3(60, 7, Steer_Duty[3]);
  lcd_refresh();
}
void display_ad_up()
{
	ad_coeff++;
	if(ad_coeff<1)	ad_coeff=100;
}
void display_ad_down()
{
	ad_coeff--;
}
void display_ad_ok()
{
	ad_coeff=ad_coeff*400/H_middle;
}

/*
	所有电感
*/

void display_add()
{
	user_lcd_cls();
	user_lcd_p6x8num4(0, 0, V_left);
	//user_lcd_p6x8str(62,0,"|");
	user_lcd_p6x8num4(92, 0, V_right);

	user_lcd_p6x8num4(0, 2, H_left);
	//user_lcd_p6x8str(62,2,"-");
	user_lcd_p6x8num4(92, 2, H_right);

	user_lcd_p6x8num4(50,2,H_middle);
	
	user_lcd_p6x8num4(0, 4, H_1);
    user_lcd_p6x8num4(50,4,ring_ad_max);
	user_lcd_p6x8num4(92, 4, H_5);
	user_lcd_p6x8num4(0, 6, H_2);
	user_lcd_p6x8num4(92, 6, H_4);
	user_lcd_p6x8num4(50,6,H_3);
}

/*
	双车通信接收到的数据
*/
void display_receive(void)
{
  user_lcd_cls();
  user_lcd_p6x8str(0,0,"Dis_H");
  user_lcd_p6x8str(0,2,"F_or_A");
  user_lcd_p6x8str(0,4,"Dis_L");
  user_lcd_p6x8str(0,6,"Stop_F");
  user_lcd_p6x8num4(75,0,Receive_Data1[0]);
  user_lcd_p6x8num4(75,2,Receive_Data1[1]);
  user_lcd_p6x8num4(75,4,Receive_Data1[2]);
  user_lcd_p6x8num4(75,6,Receive_Data1[3]);  
}

/*
	模糊转向参数p
*/
//void display_fuzzy_p()
//{
//	int i;
//	user_lcd_cls();
//	user_lcd_p6x8str(0, 0, "p");
//	user_lcd_p6x8num4(12, 0, Dir_Kp);
//	user_lcd_p6x8str(64, 0,"d");
//	user_lcd_p6x8num4(76, 0, Dir_Kd/10);
//	user_lcd_p6x8str(0, 1,"parameter p");
//	user_lcd_p6x8str(18, 2, "ZO");
//	user_lcd_p6x8num4(80, 2, fuzzy_p_params_list[MODE-1][0]);
//	user_lcd_p6x8str(18, 3, "SS");
//	user_lcd_p6x8num4(80, 3, fuzzy_p_params_list[MODE-1][1]);
//	user_lcd_p6x8str(18, 4, "MM");
//	user_lcd_p6x8num4(80, 4, fuzzy_p_params_list[MODE-1][2]);
//	user_lcd_p6x8str(18, 5, "BB");
//	user_lcd_p6x8num4(80, 5, fuzzy_p_params_list[MODE-1][3]);
//	user_lcd_p6x8str(18, 6, "VB");
//	user_lcd_p6x8num4(80, 6, fuzzy_p_params_list[MODE-1][4]);
//
//	if (ok_flag == 0)
//	{
//		for (i = 0; i<dis_stack[gui_page].row_max; i++)
//		{
//			if (i + 1 == gui_row)
//				user_lcd_p6x8str(0, i + 2, "* ");
//			else
//				user_lcd_p6x8str(0, i + 2, "  ");
//		}
//	}
//	else
//	{
//		for (i = 0; i<dis_stack[gui_page].row_max; i++)
//		{
//			if (i + 1 == gui_row)
//				user_lcd_p6x8str(0, i + 2, "**");
//			else
//				user_lcd_p6x8str(0, i + 2, "  ");
//		}
//	}
//
//	lcd_refresh();
//
//}
//void display_fuzzy_p_up()
//{
//	if (ok_flag == 0)
//	{
//		default_up_fun();
//	}
//	else
//	{
//		fuzzy_p_params_list[MODE-1][gui_row - 1] = fuzzy_p_params_list[MODE-1][gui_row - 1] + 1;
//		if (fuzzy_p_params_list[MODE-1][gui_row - 1] < 2)
//		{
//			switch(gui_row - 1)
//			{
//				case 0:fuzzy_p_params_list[MODE-1][gui_row - 1] = 28;break;
//				case 1:fuzzy_p_params_list[MODE-1][gui_row - 1] = 30;break;
//				case 2:fuzzy_p_params_list[MODE-1][gui_row - 1] = 36;break;
//				case 3:fuzzy_p_params_list[MODE-1][gui_row - 1] = 38;break;
//				case 4:fuzzy_p_params_list[MODE-1][gui_row - 1] = 40;break;
//				default:
//				break;
//			}
//		}
//	}
//}
//
//void display_fuzzy_p_down()
//{
//	if (ok_flag == 0)
//	{
//		default_down_fun();
//	}
//	else
//	{
//		fuzzy_p_params_list[MODE-1][gui_row - 1] = fuzzy_p_params_list[MODE-1][gui_row - 1] - 1;
//	}
//
//}
//
//
///*
//	模糊转向参数d
//*/
//void display_fuzzy_d()
//{
//	int i;
//	user_lcd_cls();
//	user_lcd_p6x8str(0, 0, "p");
//	user_lcd_p6x8num4(12, 0, Dir_Kp);
//	user_lcd_p6x8str(64, 0,"d");
//	user_lcd_p6x8num4(76, 0, Dir_Kd/10);
//	user_lcd_p6x8str(0, 1,"parameter d");
//	user_lcd_p6x8str(18, 2, "ZO");
//	user_lcd_p6x8num4(80, 2, fuzzy_d_params_list[MODE-1][0]);
//	user_lcd_p6x8str(18, 3, "SS");
//	user_lcd_p6x8num4(80, 3, fuzzy_d_params_list[MODE-1][1]);
//	user_lcd_p6x8str(18, 4, "MM");
//	user_lcd_p6x8num4(80, 4, fuzzy_d_params_list[MODE-1][2]);
//	user_lcd_p6x8str(18, 5, "BB");
//	user_lcd_p6x8num4(80, 5, fuzzy_d_params_list[MODE-1][3]);
//	user_lcd_p6x8str(18, 6, "VB");
//	user_lcd_p6x8num4(80, 6, fuzzy_d_params_list[MODE-1][4]);
//
//	if (ok_flag == 0)
//	{
//		for (i = 0; i<dis_stack[gui_page].row_max; i++)
//		{
//			if (i + 1 == gui_row)
//				user_lcd_p6x8str(0, i + 2, "* ");
//			else
//				user_lcd_p6x8str(0, i + 2, "  ");
//		}
//	}
//	else
//	{
//		for (i = 0; i<dis_stack[gui_page].row_max; i++)
//		{
//			if (i + 1 == gui_row)
//				user_lcd_p6x8str(0, i + 2, "**");
//			else
//				user_lcd_p6x8str(0, i + 2, "  ");
//		}
//	}
//
//	lcd_refresh();
//
//}
//void display_fuzzy_d_up()
//{
//	if (ok_flag == 0)
//	{
//		default_up_fun();
//	}
//	else
//	{
//		fuzzy_d_params_list[MODE-1][gui_row - 1] = fuzzy_d_params_list[MODE-1][gui_row - 1] + 1;
//		if (fuzzy_d_params_list[MODE-1][gui_row - 1] < 2)
//		{
//			switch(gui_row - 1)
//			{
//				case 0:fuzzy_d_params_list[MODE-1][gui_row - 1] = 135;break;
//				case 1:fuzzy_d_params_list[MODE-1][gui_row - 1] = 135;break;
//				case 2:fuzzy_d_params_list[MODE-1][gui_row - 1] = 120;break;
//				case 3:fuzzy_d_params_list[MODE-1][gui_row - 1] = 65;break;
//				case 4:fuzzy_d_params_list[MODE-1][gui_row - 1] = 65;break;
//				default:
//				break;
//			}
//		}
//	}
//}
//
//void display_fuzzy_d_down()
//{
//	if (ok_flag == 0)
//	{
//		default_down_fun();
//	}
//	else
//	{
//		fuzzy_d_params_list[MODE-1][gui_row - 1] = fuzzy_d_params_list[MODE-1][gui_row - 1] - 1;
//	}
