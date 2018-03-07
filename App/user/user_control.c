/*
* @file			user_control.c
* @brief		��вɼ�������
* @author		Benson
* @date			20161229
*/

#include "include.h"
#include "math.h"

//int16 angle=400;
extern uint8 Receive_Data1[6];
extern double UartData[9];
extern int8 front_or_after;	//ǰ�󳵱�־λ
extern int8 last_front_or_after;
extern int8 RING_WAIT_FLAG;//�����ȴ���־λ
extern int8 shift_gear;
extern uint8 RING_FLAG;//������־λ
extern uint8 LOCK_Cnt;//������������λ
extern uint8 RING_LOCK_FLAG;//����������־λ
extern uint8 Ring_Cnt;
extern int16 ring_td;
extern int16 ring_th;
extern int16 ring_tl;
extern int16 ring_adh;
extern int16 ring_adl;
extern uint16 Distance[3];
extern TIME sys_time;//���뻷ʱ�̶�Ӧ��ϵͳʱ��
TIME time_enter_ring;//���뻷��ʱ��
TIME time_exit_ring; //�˳�����ʱ��
TIME time_enter_ramp;
TIME time_exit_ramp;
TIME time_enter_cross;
uint16 R_s;
extern TIME time_launch;
extern TIME time_ring_launch;
extern int16 ring_angle[4];
extern int16 Set_Speed;
extern int8 Stop_Flag;
int8 ad_check_flag=0;
int8 demacate_flag;
int8 ring_dir,last_ring_dir;
int8 new_lock_flag;
int8 Out_Flag;
int8 Ramp_Flag=0;
int8 Over_Flag=0;
int8 Cross_Flag=0;
int8 rank_result1=0;
int8 rank_result2=0;
int16 Dir_Err[4]={0,0,0,0};
int32 chabihe,daoshucha,gongshi;
extern int8 big_ring_pos;
extern int8 ring_ad_max_START_FLAG;
extern uint8 lock_time;
uint16 sum[10];
uint16 AD_Voltage[10][10];	//����ADת����ֵ
uint16 ad_rec[5];
uint16 ring_ad_max=200;
uint16 H_left,H_right,H_middle,V_left,V_right,Z_middle,ml,mr,m;
uint16 H_1,H_2,H_3,H_4,H_5;

uint16 front_max,back_min;	//ǰ�ŵ�����ֵ�����ŵ����Сֵ
uint16 del_sum;

int16 H_sum,H_del,V_sum,V_del;
int16 ll_1,rr_1,mm_1;
int32 V_chabihe;
int8 MODE=0;
int8 Road_Type=0,Last_Road_Type=0;	


//extern int16 fuzzy_p_params_list[7][5];
//extern int16 fuzzy_d_params_list[7][5];
int16 cons=720;
uint16 ad_coeff = 100;
//int16 weaken_coeff = 100;
extern int16 Dir_Kp;
extern int16 Dir_Kd;
extern uint8 Overtake_ctrl_table[];
extern int16 Set_Speed_Temp;


#if CAR_NUM==1
uint16 Hori_ADMiddle_max=310;
//uint16 H_middle_target=433;
#elif CAR_NUM==2
uint16 Hori_ADMiddle_max=330;
//uint16 H_middle_target=433;

#elif CAR_NUM==3
uint16 Hori_ADMiddle_max=307;
//uint16 H_middle_target=433;
#endif

/*
������	�����	�����

�ҳ���	ֱ��	�����

�ҳ���	�ҹ���	������
*/

//����ȫ������һ���ļ�����
extern FUZTAB SteerFuzzyTAB0;
extern FUZTAB SteerFuzzyTAB1;
extern FUZTAB SteerFuzzyTAB2;
extern FUZTAB SteerFuzzyTAB3;
extern FUZTAB SteerFuzzyTAB4;
extern FUZTAB SteerFuzzyTAB5;
extern FUZTAB SteerFuzzyTAB6;
extern FUZTAB SteerFuzzyTAB7;

extern FUZTAB SpeedFuzzyTAB;
/*
*  @brief		AD��ʼ��
*  @author		Benson
*  @date		20161229
*/

void AD_Init()
{
	adc_init(ADC1_SE12);	//B6
	adc_init(ADC1_SE13);	//B7
	adc_init(ADC1_SE10);	//B4
	adc_init(ADC1_SE11);	//B5
	adc_init(ADC0_SE12);	//B2
	adc_init(ADC0_SE13);	//B3
	adc_init(ADC0_SE8);		//B0
	adc_init(ADC0_SE9);		//B1
	adc_init(ADC0_SE11);	//A8
	adc_init(ADC0_SE10);	//A7
}
/*
*  @brief		��вɼ�����
*  @author		Benson
*  @date		20161229
*/

void get_voltage()
{
	int i=0,j=0;
	int32 all_sum;
	int temp=0;
	for(i=0;i<10;i++)
	{
		AD_Voltage[0][i] = (uint16)adc_once(ADC1_SE12, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[1][i] = (uint16)adc_once(ADC1_SE13, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[2][i] = (uint16)adc_once(ADC1_SE10, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[3][i] = (uint16)adc_once(ADC1_SE11, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[4][i] = (uint16)adc_once(ADC0_SE12, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[5][i] = (uint16)adc_once(ADC0_SE13, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[6][i] = (uint16)adc_once(ADC0_SE8, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[7][i] = (uint16)adc_once(ADC0_SE9, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[8][i] = (uint16)adc_once(ADC0_SE11, ADC_12bit)*ad_coeff/1000;
		AD_Voltage[9][i] = (uint16)adc_once(ADC0_SE10, ADC_12bit)*ad_coeff/1000;
	}

	for(i=0;i<10;i++)
	{
		for(j=0;j<9;j++)
		{
			if(AD_Voltage[i][j]<AD_Voltage[i][j+1])
			{
			temp=AD_Voltage[i][j+1];
			AD_Voltage[i][j+1]=AD_Voltage[i][j];
			AD_Voltage[i][j]=temp;
			}
		}
		AD_Voltage[i][9]=0;	//ȥ����С
		for(j=0;j<9;j++)
		{
			if(AD_Voltage[i][j]>AD_Voltage[i][j+1])
			{
			temp=AD_Voltage[i][j+1];
			AD_Voltage[i][j+1]=AD_Voltage[i][j];
			AD_Voltage[i][j]=temp;
			}
		}
		AD_Voltage[i][9]=0;//ȥ�����
		for(j=0;j<10;j++)
		{
			sum[i]+=AD_Voltage[i][j];
		}
		sum[i]/=8;
	}
	
	//�޸ĺ��
	H_2=(uint16)(sum[0]);
	H_left=(uint16)(sum[1]);
	H_middle=(uint16)(sum[2]);
	H_right=(uint16)(sum[3]);
	H_4=(uint16)(sum[4]);
	H_1=(uint16)(sum[5]);
	V_left=(uint16)(sum[6]);
	H_3=(uint16)(sum[7]);
	V_right=(uint16)(sum[8]);
	H_5=(uint16)(sum[9]);
	
    if(H_middle>380
       && H_left>180 && H_right>180 
       && H_3>400
       && H_1>180 && H_5>180 
       )
    {
        ad_check_flag=1;
    }
    else
        ad_check_flag=0;
        
    
	ad_rec[0] = ad_rec[1];
	ad_rec[1] = ad_rec[2];
	ad_rec[2] = ad_rec[3];
	ad_rec[3] = ad_rec[4];
	ad_rec[4] = H_3;

	if (RING_FLAG)
	{
		if (demacate_flag == 0)	//��û�궨��
		{
			if (
				ad_rec[0]<ad_rec[1]
				&& ad_rec[1]<ad_rec[2]
				&& ad_rec[2]>ad_rec[3]
				&& ad_rec[3]>ad_rec[4])
			{
				ring_ad_max = ad_rec[2];
                if(ring_ad_max<224)
                    ring_ad_max=224;
                
				demacate_flag = 1;
			}
		}
	}

//ԭ����
//	V_left=(uint16)(sum[0]);
//	H_left=(uint16)(sum[1]);
//	H_middle=(uint16)(sum[2]);
//	H_right=(uint16)(sum[3]);
//	V_right=(uint16)(sum[4]);
//	H_1=(uint16)(sum[5]);
//	H_2=(uint16)(sum[6]);
//	H_3=(uint16)(sum[7]);
//	H_4=(uint16)(sum[8]);
//	H_5=(uint16)(sum[9]);
 
	/********************/
	
	//ǰ������ˮƽ������������
	
	if(H_middle>H_left)
	{
		if(H_middle>H_right)	//�����
		{
			rank_result1=0;
		}
		else					//�����
		{
			rank_result1=1;
		}
	}
	else
	{
		if(H_left>H_right)		//�����	
		{
			rank_result1=-1;
		}
		else					//�����
		{
			rank_result1=1;
		}
	}
	
	if(H_1>H_3)			//����3���������
	{
		if(H_1<H_5)		//�����
			rank_result2=1;
		else			//�����
			rank_result2=-1;
	}
	else
	{
		if(H_3<H_5)		//�����
			rank_result2=1;
		else			//�����
			rank_result2=0;
	}
	
	/********************/
	
	for(i=0;i<4;i++)	//ǰ��5���������
	{
		if(sum[i]>sum[i+1])
		{
			temp=sum[i];
			sum[i]=sum[i+1];
			sum[i+1]=temp;
		}
	}
	front_max=sum[4];	//ȡǰ�����
	
	if(H_1>H_3)			//����3���������
	{
		if(H_3<H_5)
			back_min=H_3;
		else
			back_min=H_5;
	}
	else
	{
		if(H_1<H_5)
			back_min=H_1;
		else
			back_min=H_5;
	}	//ȡ������С
	
	/********************/
	//push(4,back_min-front_max);
	
	push(9,V_left);
	push(10,H_left);
	push(11,H_middle);
	push(12,H_right);
	push(13,V_right);
	push(14,H_1);
	push(15,H_3);
	push(16,H_5);

	H_sum=H_left+H_right;
	H_del=H_left-H_right;
	V_sum=V_left+V_right;
	V_del=V_left-V_right;
  
	all_sum=0;
	for(i=0;i<10;i++)
	{
		all_sum+=sum[i];
	}
	if(all_sum<5 && Stop_Flag==0 && Road_Type!=3 && Road_Type!=4 && Road_Type!=5)
	{
		Stop_Flag=1;
		beep_on();
		user_oled_delay(8000);
		beep_off();
	}
	
//	ml=H_left*1000/(H_middle+1);
//	mr=H_right*1000/(H_middle+1);
	
//	ml=abs(H_middle-H_left);
//	mr=abs(H_middle-H_right);
	
	ml=H_middle-H_left;
	mr=H_middle-H_right;
	
//	push(1, ml);
//	push(2, mr);
//	push(3, abs(H_left-H_right));
	
	
	m=(V_left>V_right)?(V_left*1000/V_right):(V_right*1000/V_left);
	ll_1=H_1*100/H_left;
	rr_1=H_5*100/H_right;
	mm_1=H_3*100/H_middle;
}


/*
*  @brief		·������
*  @author		Benson
*  @date		20161229
*/
void road_analyze()
{
//	double x;
//	int32 y;

  	Last_Road_Type=Road_Type;
	
	//�µ�
    
    R_s=get_timevar_ms(sys_time,time_enter_ramp);
    if(Ramp_Flag==0 
       //&& H_middle>500
       && (H_middle+H_left>870 || H_middle+H_right>870 )
       && abs(H_middle-H_left)>80 && abs(H_middle-H_right)>80
       //&& H_middle>500 && V_sum<150
       )
    {
      Ramp_Flag=1;
      time_enter_ramp=sys_time;
    }
    if(Ramp_Flag==1 && R_s>1200)
    {
      Ramp_Flag=0;
    }
	
	if(Ramp_Flag)
	{
		beep_on();
	}
	else
	{
		beep_off();
	}
	
	
	//��ʮ��
	if (RING_FLAG == 0 && Cross_Flag == 0 && Ramp_Flag == 0		//·������
		&& V_sum > 700
		&& H_sum>400
		&& V_del<70 && V_del>-70
		&& get_timevar_ms(sys_time, time_exit_ring) > 800		//����ʱ�����1s�����ж�ʮ�֣���������ж�ʮ��
		)
	{
		Cross_Flag=1;
		time_enter_cross=sys_time;
	}	
	//��ʮ��
	if(Cross_Flag==1&&((V_left<200)&&(V_right<200))&&((H_del>30)||(H_del<-30)))
	{
		Cross_Flag=0;
	}
  
	//�����뻷��	ʹ����ǰ�����ţ�bmi-fma
//	if (Cross_Flag == 0 && RING_FLAG == 0
//		&& get_timevar_ms(sys_time, time_exit_ring) > 1000
//		&& (H_1 + H_3 + H_5) > 900 && (H_1 + H_3 + H_5) < 1150
//		&& (ll_1 > 160 && ll_1 < 230) && (rr_1 > 160 && rr_1 < 230)
//		&& (mm_1 > 140 && mm_1 < 195)
//		&& (Dir_Err[3]<300 && Dir_Err[3]>-300))
	
//		if(front_max<back_min && back_min-front_max>ring_tl
//		   && H_left-H_right>-ring_td && H_left-H_right<ring_td
//			&&V_left+V_right>ring_adl
//			&& V_left-V_right>-ring_th  && V_left-V_right<ring_th
//		   && Road_Type==0
//		  	&&Cross_Flag==0&&RING_FLAG==0
//	       	&& get_timevar_ms(sys_time,time_exit_ring)>1000	//ʱ��������������������������ж��뻷
//		   )	

	
	//�����Ⱥ�
//	if(H_sum>200)
//		chabihe = (int32)(((float)(H_right - H_left)) * 1000 / ((float)(H_left + H_right)));
//	x = (double)H_middle;
//	y = formula2(x);
//
//	push(1, (int16)y);
//	push(2, Dir_Err[3]);
//	push(3, abs(chabihe));
	//	���ò�Ⱥ��뵥��еĲ�࣬Ч����Թ�ʽformula2Ҫ�󼫸�
//	if ((y - abs(chabihe)>200)			//���߲�ֵ�ﵽ��ֵ
//		&& abs(chabihe)<300				//��Ⱥ���õ�ƫ���㹻С
//		&& y>310						//�м�����õ�ƫ���㹻��
//		//��������Ҫ����
//
//		//���漸����һ������������һ�㲻���и���
//		&& abs(Dir_Err[3])<500		//�������
//		&& abs(Dir_Err[2])<500
//		&& abs(Dir_Err[1])<500
//		&& abs(Dir_Err[0])<500
//			
//		&& H_sum+V_sum>100			//û�źŵĵط�
//		&& V_left<100 && V_right<100	//һЩ������
//
//		&& Cross_Flag == 0 && Ramp_Flag == 0 && RING_FLAG == 0	//�⼸��·����ʱ�������жϻ���
//		&& get_timevar_ms(sys_time, time_exit_ring)>1000		//ʱ��������������һ��ʱ������ж��뻷
//		&& sys_time.ss>2				//���⸴λ���
//		)

	
//	��ֵ
//    if(Cross_Flag==0&&RING_FLAG==0
//       && get_timevar_ms(sys_time,time_exit_ring)>2000	//ʱ��������������������������ж��뻷
//         &&H_middle<ring_adh && H_middle>ring_adl
//           &&(ml-mr<ring_td && ml-mr>0-ring_td)
//             &&(ml>ring_th || mr>ring_th)&&(ml>ring_tl && mr>ring_tl))
		
//6��25��
//    if(	Ramp_Flag==0 && Cross_Flag==0&&RING_FLAG==0
//	   	&&rank_result1==0 && (back_min==H_1 || back_min==H_5)
//	   	&&front_max<back_min &&(back_min-front_max)>20
//		&& get_timevar_ms(sys_time,time_exit_ring)>1000
//		&& abs(Dir_Err[3]<100)
//		&&(abs(ml-mr)<ring_td)
//		&&(ml>ring_th || mr>ring_th)
//		&&(ml>ring_tl && mr>ring_tl)
//		&&V_sum<120
//			)
	
	if(	Ramp_Flag==0 && Cross_Flag==0&&RING_FLAG==0
	   	&&rank_result1==0
		//&& (back_min==H_1 || back_min==H_5)
	   	&& front_max<back_min &&(back_min-front_max)>0
		&& get_timevar_ms(sys_time,time_exit_ring)>800
		//&& abs(Dir_Err[3]<100)
		//&&(abs(H_left-H_right)<ring_td)
		//&&(ml<ring_th && mr<ring_th)
		//&&(ml<ring_tl || mr<ring_tl)
		
//		&& H_middle<ring_adh
//		&& H_middle>ring_adl
        && H_middle<250
		&& H_middle>90
			
		//&&H_middle>80
		&&V_sum<120
		&& get_timevar_ms(sys_time,time_enter_cross)>600
			)
//	if(0)
	{
		//��¼ʱ��
		time_enter_ring = sys_time;
        if(Ring_Cnt==big_ring_pos-1)
        {
            lock_time=142;
        }
        else
        {
            lock_time=80;
        }
		//�뻷������¼����ǰ˳λ
		#if OVERTAKE
		if(Overtake_ctrl_table[Ring_Cnt])
		{
			last_front_or_after= front_or_after;
			front_or_after=0;
		}
		#endif
        new_lock_flag=1;
		RING_FLAG=1;
	}


	//��������
	if (RING_FLAG == 1 && LOCK_Cnt > 0
		&&((V_sum > 200 && H_sum > 210)||H_middle>340 || V_sum>400)
        //&& (V_del > 30 || V_del < -30)
		&& get_timevar_ms(sys_time, time_enter_ring) > 300	//ʱ���������뻷�����ٰ�������жϳ���		
		//&& ((H_left+H_right)>300 || (V_left+V_right)<150)
			)
	{
		RING_FLAG = 0;
		LOCK_Cnt = 0;		//�����������
	//����������ǰ���־λ
    
		time_exit_ring = sys_time;	//��¼ʱ��
#if OVERTAKE
		if(Overtake_ctrl_table[Ring_Cnt])
		{
			if(last_front_or_after==1)	//last_front_or_after�˴�����¼0
				front_or_after=2;
			if(last_front_or_after==2)
				front_or_after=1;
		}
#endif
		demacate_flag = 0;	
		Ring_Cnt++;	//��������
		if(Ring_Cnt>=Ring_Cnt_max)	//��󻷵�����
			Ring_Cnt=0;
	}
//ֱ��
	///////////////////////////////ǰ���ۺ�����
	if(H_left+H_right+H_middle>120)
	{
		Road_Type=rank_result1;
	}
	else
	{
		if(H_1+H_3+H_5>100)
		{
			if(H_1>H_5+15)
				Road_Type=-2;
			else if(H_5>H_1+15)
				Road_Type=2;
			else
				Road_Type=Last_Road_Type;
		}
		else
		{
			if (Last_Road_Type != 4 && Last_Road_Type != 5 && Last_Road_Type != 3 && Last_Road_Type != -3)
				Road_Type=Last_Road_Type;
			else
				Road_Type=rank_result2*2;
		}
	}
	/////////////////////////////////////////////////////////

//	�������µ�·���ж������������޸���6��24��
//	/////////////////////////////
//	һ�����
//	if(H_left+H_right>280)	//ֱ��
//	{
//		Road_Type=0;
//	}
//	else					//С��400
//	{
//		if(H_left-H_right<30 && H_left-H_right>-30)		//��ֵ̫С�����ɿ�
//		{
//			if(Last_Road_Type!=3 && Last_Road_Type!=4 && Last_Road_Type!=-3 && Last_Road_Type!=5
//		   		&& Last_Road_Type!=0)
//			{
//				Road_Type=Last_Road_Type;
//			}
//			else		//ǿ����һ������
//			{
//				if(H_left>H_right)
//				{
//					Road_Type=-1;
//				}
//				else
//				{
//					Road_Type=1;
//				}
//			}
//		}
//		else
//		{
//			if(H_left>H_right)
//			{
//				Road_Type=-1;
//			}
//			else
//			{
//				Road_Type=1;
//			}
//		}
//	}
//	
//	if(H_sum<180
//	   && Last_Road_Type!=3 && Last_Road_Type!=4 && Last_Road_Type!=-3 && Last_Road_Type!=5
//		   && Last_Road_Type!=0		//�����ﲻ�ܱ���0
//		)
//	{
//		Road_Type=Last_Road_Type;
//	}

	/////////////////////////////////
	
	if(Cross_Flag)
	{
		if(V_del>0)
			Road_Type=-3;
		else
			Road_Type=3;
	}
	if(RING_FLAG==1)
	{
		Road_Type=4;
		if(H_middle<300 && LOCK_Cnt==0)
		{
			Road_Type=5;
			RING_LOCK_FLAG=1;	//��ʱ��ʼ
		}
	}
	push(0,Road_Type*100);
}
/*
	ģ������ļ���
*/
void fuzzy_steer_control()
{    
	int16 error;
	int16 errorC;
	int16 kp=0,ki=0,kd=0; 
	error=Dir_Err[3];
	errorC=Dir_Err[3]-Dir_Err[2];	
	switch(MODE)
	{
		case 1:fuzzyout(&SteerFuzzyTAB1, error, errorC, &kp, &ki, &kd);
		break;
		case 2:fuzzyout(&SteerFuzzyTAB2, error, errorC, &kp, &ki, &kd);
		break;
		case 3:fuzzyout(&SteerFuzzyTAB3, error, errorC, &kp, &ki, &kd);
		break;
		case 4:fuzzyout(&SteerFuzzyTAB4, error, errorC, &kp, &ki, &kd);
		break;
		case 5:fuzzyout(&SteerFuzzyTAB5, error, errorC, &kp, &ki, &kd);
		break;
		case 6:fuzzyout(&SteerFuzzyTAB6, error, errorC, &kp, &ki, &kd);
		break;
		case 7:fuzzyout(&SteerFuzzyTAB7, error, errorC, &kp, &ki, &kd);
		break;
		default:fuzzyout(&SteerFuzzyTAB0, error, errorC, &kp, &ki, &kd);
		break;
	}
	Dir_Kp=kp/10;
	Dir_Kd=kd;
}
/*
void steer_param_interpolation(int16 speed)
{
    int16 speed_tab[7]={65,80,85,88,92,96,102};
    int16 param_p[7];
    int16 param_d[7];
    int16 kp,ki,kd;
    int8 i;
    int8 l_node,r_node;
    
    fuzzyout(&SteerFuzzyTAB1, error, errorC, &kp, &ki, &kd);
    param_p[0]=kp;param_d[0]=kd;
    fuzzyout(&SteerFuzzyTAB2, error, errorC, &kp, &ki, &kd);
    param_p[1]=kp;param_d[1]=kd;
    fuzzyout(&SteerFuzzyTAB3, error, errorC, &kp, &ki, &kd);
    param_p[2]=kp;param_d[2]=kd;
    fuzzyout(&SteerFuzzyTAB4, error, errorC, &kp, &ki, &kd);
    param_p[3]=kp;param_d[3]=kd;
    fuzzyout(&SteerFuzzyTAB5, error, errorC, &kp, &ki, &kd);
    param_p[4]=kp;param_d[4]=kd;
    fuzzyout(&SteerFuzzyTAB6, error, errorC, &kp, &ki, &kd);
    param_p[5]=kp;param_d[5]=kd;
    fuzzyout(&SteerFuzzyTAB7, error, errorC, &kp, &ki, &kd);
    param_p[6]=kp;param_d[6]=kd;
    
    if(speed>speed_tab[6])
    {
        Dir_Kp=param_p[6];
        Dir_Kd=param_d[6];
    }
    else if(speed<speed_tab[0])
    {
        Dir_Kp=param_p[0];
        Dir_Kd=param_d[0];
    }
    else
    {
        for(i=1;i<7;i++)
        {
            if(speed<speed_tab[i])
            {
                l_node=i-1;
                r_node=i;
            }
            Dir_Kp=((speed-speed_tab[l_node])*param_p[r_node]+(speed_tab[r_node]-speed)*param_p[l_node])/(speed_tab[r_node]-speed_tab[l_node]);
            Dir_Kd=((speed-speed_tab[l_node])*param_d[r_node]+(speed_tab[r_node]-speed)*param_d[l_node])/(speed_tab[r_node]-speed_tab[l_node]);
        }
    }
}
*/


/*
*  @brief		ƫ�����
*  @author		Benson
*  @date		20161229
*/
void err_analyze()
{
	double x;
//	int32 y;
	Dir_Err[0]=Dir_Err[1];
	Dir_Err[1]=Dir_Err[2];
	Dir_Err[2]=Dir_Err[3];
  
	chabihe = (int32)(((float)(H_left - H_right))*(-1000) / ((float)(H_left + H_right)));//��Ⱥ�
	//V_chabihe = (int32)(((float)(V_left - V_right))*(-1000) / ((float)(V_left + V_right)));//��ֱ���
	//push(5, V_chabihe);
  
	x=(double)H_middle;
	//gongshi=(int32)(0.004475*x*x-3.506*x+cons);
	//gongshi����������ɺ���formula1����
	//��ͨ���
	if(Road_Type>0)
	{
		Dir_Err[3]=formula1(x)+50;
	}

	else if(Road_Type<0)
	{
		Dir_Err[3]=-formula1(x)-50;
	}
	else	//ֱ��
	{
		Dir_Err[3]=chabihe*22/100;
	}
	
	//�µ���ʮ������ͬ����
	//����������жϷ��򣬾Ͳ�����
//	if(Ramp_Flag)
//	{
//		Dir_Err[3]=chabihe*12/50;
//	}
	//ʮ����ˮƽ��в�Ⱥ�
    if(Road_Type==3 || Road_Type==-3)	
    {
		Dir_Err[3]=chabihe*12/50;	//ʮ�ֵ�����ϵ��
    }
    
	//����
    if(Road_Type==4)
    {  
		if(RING_LOCK_FLAG==0)	//��������״̬
		{
			if(LOCK_Cnt==0)	//��û�������������ռ�⵽����
			{
				;//�����Ѽ�⵽������������������䲻����
			}
			else	//�Ѿ��������ˣ��ò�Ⱥ�
			{
#if CAR_NUM==1
				last_ring_dir = ring_dir;
				if (H_1 + H_5 > 85)
				{
					if (H_1 > H_5)
						ring_dir = -1;
					else if (H_5 > H_1)
						ring_dir = 1;
					else
						ring_dir = last_ring_dir;
				}
				else
				{
					ring_dir = last_ring_dir;
				}
			/*	if(Ring_Cnt==big_ring_pos-1)
				{
					if (ring_dir > 0)
					{
						Dir_Err[3] = formula3(H_3, ring_ad_max);
					}
					else
					{
						Dir_Err[3] = -formula3(H_3, ring_ad_max);
					} 
				else
				{
					if (ring_dir > 0)
					{
						Dir_Err[3] = formula3(H_3, 250);
					}
					else
					{
						Dir_Err[3] = -formula3(H_3,250);
					}
				}*/
                                
                                
				if (ring_dir > 0)
				{
                    if(Ring_Cnt==big_ring_pos-1)
                        Dir_Err[3] = formula1((double)H_3);
                    else
                        Dir_Err[3] = formula1((double)H_3);
				}
				else
				{
					if(Ring_Cnt==big_ring_pos-1)
                        Dir_Err[3] = -formula1((double)H_3);
                    else
                        Dir_Err[3] = -formula1((double)H_3);
				}
#elif CAR_NUM==2
                                
                if(Ring_Cnt==big_ring_pos-1)            //��
                {
                    last_ring_dir = ring_dir;
                    if (H_1 + H_5 > 85)
                    {
                        if (H_1 > H_5)
                                ring_dir = -1;
                        else if (H_5 > H_1)
                                ring_dir = 1;
                        else
                                ring_dir = last_ring_dir;
                    }
                    else
                    {
                        ring_dir = last_ring_dir;
                    }
                    if (ring_dir > 0)
                    {
                        if(Ring_Cnt==big_ring_pos-1)
                            Dir_Err[3] = formula1((double)H_3);
                        else
                            Dir_Err[3] = formula1((double)H_3);
                    }
                    else
                    {
                        if(Ring_Cnt==big_ring_pos-1)
                            Dir_Err[3] = -formula1((double)H_3);
                        else
                            Dir_Err[3] = -formula1((double)H_3);
                    }
                }
                else            //С��
                {
                    if(H_1+H_5>85)
                    {
                        Dir_Err[3]=(int32)(((float)(H_1-H_5))*(-1235)/((float)(H_1+H_5)));//*78/100;//��Ⱥ�
                        if(Dir_Err[3]>0 && Dir_Err[3]<300)
                            Dir_Err[3]=300;
                        if(Dir_Err[3]>0 && Dir_Err[3]>900)
                            Dir_Err[3]=900;
                                            
                                            
                        if(Dir_Err[3]<0 && Dir_Err[3]>-300)
                            Dir_Err[3]=-300;
                        if(Dir_Err[3]<0 && Dir_Err[3]<-900)
                            Dir_Err[3]=-900;
                    }
                    else
                    {
                        Dir_Err[3] = Dir_Err[2];
                    }
                }
                                
                                
#else
                                
#endif
#if OVERTAKE
				if (Overtake_ctrl_table[Ring_Cnt])
				{
					if ((RING_WAIT_FLAG || get_timevar_ms(time_ring_launch, sys_time) < 400) && last_front_or_after == 1)
					{
						Dir_Err[3] = Dir_Err[2];
						ring_dir = last_ring_dir;
					}
				}
#endif
			}
		}
		else	//����������
		{
			//Ring_Cnt�ǳ�������+1����˵�һ��������Ring_Cnt=0
			Dir_Err[3] = ring_angle[Ring_Cnt];
		}
    }
    //�������ã���ɾ
    if (Road_Type == 5)
    {
		Dir_Err[3] = ring_angle[Ring_Cnt];
    }
    
    //��������
//    if(get_timevar_ms(sys_time,time_exit_ring)<270)
//    {
//		//Dir_Err[3]=(int32)(((float)(H_1-H_5))*(-1000)/((float)(H_1+H_5)))*75/100;//���ŵ�в�Ⱥ�
//        Dir_Err[3]=(int32)(((float)(H_left-H_right))*(-1000)/((float)(H_left+H_right)))*75/100;//ǰ�ŵ�в�Ⱥ�
//		Dir_Err[3]=Dir_Err[3]*70/100;
//    }
    
  
	if(Dir_Err[3]>1100)
			Dir_Err[3]=1100;
	if(Dir_Err[3]<-1100)
	Dir_Err[3]=-1100;
    
	push(3,Dir_Err[3]);
}


/*
*  @brief		ģ���ٶ��趨
*  @author		Benson
*  @date		20170212
*/

int16 set_speed_aim()
{
	int16 e,ec,u1,u2,u3;
	int16 speed;
	e=Dir_Err[3];
	ec=Dir_Err[3]-Dir_Err[2];
	if(e<0)
	{
		e=-e;
		ec=-ec;
	}
	fuzzyout(&SpeedFuzzyTAB, e, ec,&u1,&u2,&u3);
	switch(shift_gear)
	{
		case 1:speed=u1/100;break;
		case 2:speed=u2/100;break;
		case 3:speed=u3/100;break;
		default:speed=Set_Speed_Temp;break;
	}
	return speed;
}

/*
�ٶȿ���
*/

void speed_control()
{
  
  /**********����************/
#if SHIFT_FLAG
	if(Road_Type==4)	//������
	{
        if(big_ring_pos-1==Ring_Cnt)
            Set_Speed=70;
        else
            Set_Speed=65;
        if (get_timevar_ms(sys_time, time_enter_ring) < 270)	//�뻷����ʱ��
        {
            if(big_ring_pos-1==Ring_Cnt)
                    Set_Speed=70;
            else
                Set_Speed=65;
        }
        #if OVERTAKE
        if(RING_WAIT_FLAG && last_front_or_after==1)
                Set_Speed=0;
        #endif
	}
	else if(Road_Type==5)
	{
//		if(Set_Speed>60)	//�뻷����
//			Set_Speed-=4;
        if(big_ring_pos-1==Ring_Cnt)
            Set_Speed=70;
        else
            Set_Speed=65;
	}
	else
	{
		if (get_timevar_ms(sys_time, time_exit_ring) < 270)	//��������ʱ��
		{
//			if(Set_Speed>60)
//				Set_Speed-=4;
			if(big_ring_pos-1==Ring_Cnt)
                Set_Speed=70;
            else
                Set_Speed=65;
		}
		else
		{
            if(shift_gear==0)
                Set_Speed=Set_Speed_Temp;
            else
                Set_Speed=set_speed_aim();
		}
	}
  
  /***************������******************/
#else
	//������
	if(Road_Type==4)	//������
	{
		if(get_timevar_ms(sys_time,time_enter_ring)<350)
		{
            if(big_ring_pos-1==Ring_Cnt)
                Set_Speed=70;
            else
                Set_Speed=70;
		}
		else
        {
            if(big_ring_pos-1==Ring_Cnt)
                Set_Speed=70;
            else
                Set_Speed=70;
        }
		
		#if OVERTAKE
        if(Overtake_ctrl_table[Ring_Cnt])
        {
            if(RING_WAIT_FLAG && last_front_or_after==1)
                Set_Speed=0;
        }
		#endif
	}

	else if(Road_Type==5)
	{
//		if(Set_Speed>60)	//�뻷����
//			Set_Speed-=2;
		if(big_ring_pos-1==Ring_Cnt)
            Set_Speed=70;
        else
            Set_Speed=70;
	}
	else
	{
		if(get_timevar_ms(sys_time,time_exit_ring)<500)	//��������ʱ��
		{
//			if(Set_Speed>60)
//				Set_Speed--;
			if(big_ring_pos-1==Ring_Cnt)
                Set_Speed=75;
            else
                Set_Speed=65;
		}
		else
		{
			Set_Speed=Set_Speed_Temp;
		}
	}
#endif

	if (Ramp_Flag)		//�µ�����
	{
		Set_Speed = 75;
	}

#if RANK_FLAG
	distance_speed();
#endif
  
#if RANK_FLAG
	if(front_or_after==2)
	{
		if(get_timevar_ms(sys_time,time_launch)<5000)
		{
			if(Distance[2]<750)
				Set_Speed=0;
			else
				Set_Speed=Set_Speed_Temp;
		}
	}
#endif
	
	//ͣ�����ȼ����
	if(Stop_Flag==1)
		 Set_Speed=0;
    
     push(1,Set_Speed);
}


int32 formula1(double x)
{
	return (int32)(0.004475*x*x-3.506*x+720);
}
int32 formula2(double x)
{
	int32 y;
    y=(int32)(-0.006904*x*x-0.02485*x+0.04826+1114);
	if(y<0)
		y=0;
	return y*2/3;
}
int16 formula3(uint16 x, uint16 m)
{
	int16 err_max;
    
    if(big_ring_pos-1==Ring_Cnt)    //�󻷵�
        err_max=650;
    else                            //С����
        err_max=780;
    
    if(x<m)
        return err_max*(m - x) / m;
    else
        return 0;
}
int16 formula4(void)
{
    static int16 del,sum,err;
    sum=H_left+H_right;
    if(sum>90)
    {
        del=H_right-H_left;
    }
    err=(int16)del*1000/(sum+1)*67/100;
    if(sum<85)
    {
        if(err>0 && err<400)
            err=400;
        if(err<0 && err>-400)
            err=-400;
    }
    return err;
}
//int16 formula5()
//{
////    if(rank_result1==0 && H_left+H_right+H_middle>100)
//    if( H_middle+90>H_left && H_middle+90>H_right
//       && H_left+H_right+H_middle>100)
//        new_lock_flag=0;
//    
//    if(new_lock_flag)
//    {
//        return ring_angle[Ring_Cnt];
//    }
//    else
//    {
//        return formula4();
//    }
//    
//}
int16 formula6()
{
    int8 dir,err;
    if(ring_angle[Ring_Cnt]>0)  //�뻷��ʱ������ת
    {
        //if(H_1+H_3+H_5>80 && H_1>H_3)
        if(Dir_Err[2]>300)
            new_lock_flag=0;
    }
    else            //�뻷��ʱ������ת
    {
        //if(H_1+H_3+H_5>80 && H_1<H_3)
        if(Dir_Err[2]<-300)
            new_lock_flag=0;
    }
    
    if(new_lock_flag)   //�жϵ����Ե�֮ǰ�ķ���
    {
        dir=0;
    }
    else        //���Ե�֮�󣬷�����������ʱ���෴
    {
        if(ring_angle[Ring_Cnt]>0)
            dir=-1;
        else
            dir=1;
    }
    
    if(dir==1)
    {
        if(Ring_Cnt==big_ring_pos-1)
            err=formula3(H_3,ring_ad_max);
        else
            err=formula3(H_3,240);
    }
    else if(dir==-1)
    {
        if(Ring_Cnt==big_ring_pos-1)
            err=-formula3(H_3,ring_ad_max);
        else
            err=-formula3(H_3,240);
    }
    else
    {
        if(H_1+H_5>90)
        {
            if(H_1>H_5)
                ring_dir=-1;
            else
                ring_dir=1;
        }
        else
        {
            ring_dir=last_ring_dir;
        }
        
        if(ring_dir==1)
        {
            if(Ring_Cnt==big_ring_pos-1)
                err=formula3(H_3,ring_ad_max);
            else
                err=formula3(H_3,240);
        }
        else
        {
            if(Ring_Cnt==big_ring_pos-1)
                err=-formula3(H_3,ring_ad_max);
            else
                err=-formula3(H_3,240);
        }
    }
    return err;
}