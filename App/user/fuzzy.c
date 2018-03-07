#include "include.h"


extern int8 Road_Type;


//uint16 distance_rule1[5]={800,900,1000,1100,1200};		//距离设定为800
//uint16 distance_rule2[5]={800,90,1000,1100,1200};		//距离设定为800
//uint8 distance_Speed1[5]={100,95,90,85,80};			//前车
//uint8 distance_Speed2[5]={80,85,90,95,100};			//后车

extern int16 distance_rule1[9];
extern int16 distance_rule2[9];
extern int16 distance_Speed1[9];
extern int16 distance_Speed2[9];


void fuzzyout(const FUZTAB *l_tab,s16 l_EE,s16 l_EEC,s16 *pout,s16 *iout,s16 *dout)
{
   unsigned char  i,j;
   long psum=0,isum=0,dsum=0;
   s16 l_BP[10]={0,0,0,0,0,0,0,0,0,0};
   s16 l_BD[10]={0,0,0,0,0,0,0,0,0,0};


   //先确定偏差位置，并求出占相邻点比重
   for(i=0;i<l_tab->cntrow;i++) 
   {   
     if(l_EE<=l_tab->Edot[i])break;
   };
   //Road_Type=i;
   if(i==0)l_BP[0]=100;
   else if(i==l_tab->cntrow)l_BP[l_tab->cntrow-1]=100;		//确定偏差位于哪两个语言值之间，并分别求出对相邻两语言值的隶属度
   else if(i>0 && i<l_tab->cntrow)
   {
     l_BP[i]=(long)100*(l_EE-l_tab->Edot[i-1])/(l_tab->Edot[i]-l_tab->Edot[i-1]);
     l_BP[i-1]=(long)100*(l_tab->Edot[i]-l_EE)/(l_tab->Edot[i]-l_tab->Edot[i-1]);  
   }

   //同理求出偏差微分的位置，并求出占相邻点比重
   for(j=0;j<l_tab->cntcolume;j++) 
   {
     if(l_EEC<=l_tab->ECdot[j])break;
   };
   if(j==0)l_BD[0]=100;
   else if(j==l_tab->cntcolume)l_BD[l_tab->cntcolume-1]=100;
   else if(j>0 && j<l_tab->cntcolume) 
   {
     l_BD[j]=(long)100*(l_EEC-l_tab->ECdot[j-1])/(l_tab->ECdot[j]-l_tab->ECdot[j-1]);
     l_BD[j-1]=(long)100*(l_tab->ECdot[j]-l_EEC)/(l_tab->ECdot[j]-l_tab->ECdot[j-1]);  
   }


   //求出该点在这个二维结构中所占比重
   for(i=0;i<l_tab->cntrow;i++)
   {
     for(j=0;j<l_tab->cntcolume;j++)
     {
       psum+=(long)l_BP[i]*l_BD[j]*l_tab->ptab[i][j];
       isum+=(long)l_BP[i]*l_BD[j]*l_tab->itab[i][j];
       dsum+=(long)l_BP[i]*l_BD[j]*l_tab->dtab[i][j];
     }
   }
   *pout=(s16)(psum/100);
   *iout=(s16)(isum/100);
   *dout=(s16)(dsum/100);
}

int distance_fuzzy(int32 distance,uint8 position)
{
    uint8 i;
    uint8 j;
    uint8 distance_count[2]={0};
    int Speed_count=100;
    if(position==1)
    {
        for(i=0;i<9;i++)
        {
            if(distance<distance_rule1[i])
              break;
        }
        if(i==0)
        {
            j=1;
            distance_count[0]=100;
        }
        else if(i==9)
        {
            j=8;
            distance_count[1]=100;
        }
        else if(i>0 && i<9)
        {
            j=i;
            distance_count[0]=100*(distance_rule1[j]-distance)/(distance_rule1[j]-distance_rule1[j-1]);
            distance_count[1]=100*(distance-distance_rule1[j-1])/(distance_rule1[j]-distance_rule1[j-1]);
        }
        Speed_count=(distance_count[0]*distance_Speed1[j-1]+distance_count[1]*distance_Speed1[j])/100;
    }
    if(position==2)
    {
        for(i=0;i<9;i++)
        {
            if(distance<distance_rule2[i])
              break;
        }
        if(i==0)
        {
            j=1;
            distance_count[0]=100;
        }
        else if(i==9)
        {
            j=8;
            distance_count[1]=100;
        }
        else if(i>0 && i<9)
        {
            j=i;
            distance_count[0]=100*(distance_rule2[j]-distance)/(distance_rule2[j]-distance_rule2[j-1]);
            distance_count[1]=100*(distance-distance_rule2[j-1])/(distance_rule2[j]-distance_rule2[j-1]);
        }
        Speed_count=(distance_count[0]*distance_Speed2[j-1]+distance_count[1]*distance_Speed2[j])/100;
    }
    return Speed_count;
}