#include "include.h"
#include "common.h"

//距离参数
//从左到右是越来越远，这里是Distance-Set_Distance
//这个偏差是反着取的，从左到右对应距离依次增大，目标距离应当大于800
const int16 distance_rule1[9]={-800,-400,-200,-100,0,100,200,400,800};		//距离设定为800
const int16 distance_rule2[9]={-800,-400,-200,-100,0,100,200,400,800};		//距离设定为800
const int16 distance_Speed1[9]={106,105,104,102,100,98 ,96 ,95 ,94 };			//前车
const int16 distance_Speed2[9]={94 ,95 ,96 ,98 ,100,102,104,105,106};			//后车


const FUZTAB DisFuzzyTAB={ 5,9,
    {
  //前车
  //-800,-400,-200,-100,0 , 100,200,400,800
    {106,105, 104, 102, 100, 98 ,96 ,95 ,94 },  //0
    {105,104, 103, 102, 100, 98 ,98 ,97 ,96 },  //50
    {103,102, 102, 101, 100, 99 ,98 ,98 ,97 },  //120
    {102,102, 101, 101, 100, 99 ,99 ,98 ,98 },  //250
    {101,101, 100, 100, 100, 100,100,99 ,99 },  //400
    },  
  //后车
    { 
    {94 ,95 ,96 ,98 ,100,102,104,105,106},
    {96 ,97 ,98 ,98 ,100,102,103,104,105},
    {97 ,98 ,98 ,99 ,100,101,102,102,103},
    {98 ,98 ,99 ,99 ,100,101,101,102,102},
    {99 ,99 ,100,100,100,100,100,101,101},
    },
    {
    {100,100,100,100,100,100,100,100,100},
    {100,100,100,100,100,100,100,100,100},
    {100,100,100,100,100,100,100,100,100},
    {100,100,100,100,100,100,100,100,100},
    {100,100,100,100,100,100,100,100,100},
    },
    
    {0,100,200,300,400,500,600},
    {-800,-400,-200,-100,0,100,200,400,800},
};
/*
	不同档位对应的速度参数
*/

int16 Set_Speed_list[8] = {80,65,80,88,92,96,100,102 };

/*
	变速
	输入量是偏差及微分
*/
const FUZTAB SpeedFuzzyTAB = { 7, 7,
//	 -45, -30. -6,  0 , 6 , 30 ,45 
{ 	{ 85,  88 , 95,  103,95 , 88, 85 },	//0
	{ 85 , 88 , 92 , 96, 92 , 88, 85 },	//60
	{ 84 , 85 , 88 , 92, 88 , 85, 84 },	//120
	{ 82,  84,  85 , 88, 85 , 84, 82 },	//180
	{ 82,  82,  84 , 85, 84 , 82, 82 },	//250
	{ 82,  82,  82 , 82, 82 , 82, 82 },	//320
	{ 82,  82,  82 , 82, 82 , 82, 82 }, },	//390	

{
	{ 90, 94, 98, 106, 98, 94, 90 },	//0
	{ 89, 92, 96, 98 , 96, 92, 89 },	//60
	{ 88, 90, 92, 95 , 92, 90, 88 },	//120
	{ 88, 88, 92, 93 , 92, 88, 88 },	//180
	{ 88, 88, 92, 92 , 92, 88, 88 },	//250
	{ 87, 88, 90, 91 , 90, 88, 87 },	//320
	{ 87, 88, 89, 90 , 89, 88, 87 },
},	//390


{
	{ 102, 107, 109, 120, 107, 105, 100 },	//0
	{ 102, 106, 108, 105, 106, 104, 100 },	//60
	{ 102, 106, 106, 100, 105, 104, 100 },	//120
	{ 102, 106, 106, 100, 105, 104, 100 },	//180
	{ 99, 102, 104, 100, 103, 99, 99 },	//250
	{ 99, 99, 102, 100, 101, 99, 97 },	//320
	{ 96, 99, 101, 103, 100, 97, 96 },
},

	{ 0, 100, 150, 180, 240, 290, 360 },
	{ -15, -12, -6, 0, 6, 12, 15 },
};
