#ifndef __FLAG_H__
#define __FLAG_H__

#define FUZZY_FLAG	1	//模糊转向，1启用
#define RANK_FLAG	1	//距离控制，1为启用
#define SHIFT_FLAG	1	//变速跑，1启用
#define SHIFT_LEVEL 2	        //变速等级 1最慢
#define OVERTAKE	1	//超车
#define display_or_not  1       //0显示 1 可控显示
#define CAR_NUM		1	//1车还是2车
#define Ring_Cnt_max 3


#endif

//串口发送通道
//0		当前电流
//1		当前速度
//2		Road_Type*100
//3		Set_Current
//4		Set_Speed
//5		
//6		E
//7		EC
//8		Steer_Duty
//9		//后面8个通道是电感
//10
//11
//12  
//13
//14
//15
//16