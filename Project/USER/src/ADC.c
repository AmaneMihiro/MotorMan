#include "ADC.h"
#include "math.h"
// int16 aim_speeda        = 730;  //目标速度
int16 aim_speedb = 0; // 输出速度（动态期望速度）=期望速度*期望减速度
int16 aim_speedc = 0; // 转弯减小速度

int16 adc_value[4]; // 储存电感采集值原始值    4个电感
int16 AD_V[4];		// 储存电感采集值归一化值中间变量 （无需关心，请勿删除）
// int16 adc_max[4]={90,90,90,95}; //电感采值最大值 需要自己采集
int16 adc_max[4] = {250, 200, 175, 250};				// 电感采值最大值 需要自己采集
int16 adc_min[4] = {1, 1, 1, 1};						// 电感采值最小值  1,4,14,1
uint8 Left_Adc, Right_Adc, Left_Shu_Adc, Right_Shu_Adc; // 电感值
float adc_valueM;
int8 NM = 4; // 电感个数

// 环道参数
uint16 annulus_s;
uint16 annulus_s2; // 环岛积分距离2
uint16 annulus_t;
uint16 annulus_z;
uint16 annulus_s3 = 0; // 环岛积分距离3

struct ROAD_TYPE road_type = {0};
int16 obstacle_Current_Dir[] = {
	30,
	31,
	32,
	33,
	34,
	35,
	36,
	37,
	38,
	39,
	40,
	41,
	42,
	43,
	44,
	45,
	46,
	47,
	48,
	49,
	-69,
	-68,
	-67,
	-66,
	-65,
	-64,
	-63,
	-62,
	-61,
	-60,
	-59,
	-58,
	-57,
	-56,
	-55,
	-54,
	-53,
	-52,
	-51,
	-50,
	-49,
	-48,
	-47,
	-46,
	-45,
	-44,
	-43,
	-42,
	-41,
	-40,
	-39,
	-38,
	-37,
	-36,
	-35,
	-34,
	-33,
	-32,
	-31,
	-30,
};
/***当前位置*************/
float Current_Dir = 0;
int16 Set_gyro = 0;
int16 ADC_PWM = 0;

uint8 temp = 0;
/***************************电感采集通道初始化****************************
函数：  void ADC_int(void)
功能：  电感采值进行初始化
参数：  void
说明：  电感采集初始化
返回值；无
************************************************************************/
void ADC_int(void)
{
	adc_init(Left_ADC_Pin, ADC_SYSclk_DIV_2);	  // 初始化P0.0为ADC功能
	adc_init(LeftXie_ADC_Pin, ADC_SYSclk_DIV_2);  // 初始化P0.1为ADC功能
	adc_init(RightXie_ADC_Pin, ADC_SYSclk_DIV_2); // 初始化P0.5为ADC功能
	adc_init(Right_ADC_Pin, ADC_SYSclk_DIV_2);	  // 初始化P0.6为ADC功能

	adc_init(Mid_ADC_Pin, ADC_SYSclk_DIV_2); // 初始化P1.5为ADC功能
}

/***************************中值滤波函数*********************************
函数：uint16 adc_mid(ADCN_enum adcn,ADCCH_enum ch)
功能： 3次电感采值进行中值滤波
参数： adcn        选择ADC通道       resolution      分辨率
说明： 8位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；k(uint8)中间那个值
************************************************************************/
uint16 adc_mid(ADCN_enum adcn, ADCRES_enum ch)
{
	uint16 i, j, k, tmp;
	i = adc_once(adcn, ch);
	j = adc_once(adcn, ch);
	k = adc_once(adcn, ch);
	if (i > j)
	{
		tmp = i, i = j, j = tmp;
	}
	if (k > j)
	{
		tmp = j;
	}
	else if (k > i)
	{
		tmp = k;
	}
	else
	{
		tmp = i;
	}
	return (tmp);
}

/***************************均值滤波函数****************************
函数：  uint16 adc_ave(ADCN_enum adcn,ADCCH_enum ch,uint8 N)
功能：  中值滤波后的5个电感值求平均值
参数：  adcn        选择ADC通道
说明：  该函数调用中值滤波函数，即电感值是中位置
返回值；tmp
示例：  adc_ave(ADC_P10, ADC_8BIT)-->ADC通道为P-10，分辨率为8bit
*******************************************************************/
uint16 adc_ave(ADCN_enum adcn, ADCRES_enum ch, uint8 N)
{
	uint32 tmp = 0;
	uint8 i;
	for (i = 0; i < N; i++)
	{
		tmp += adc_mid(adcn, ch);
	}
	tmp = tmp / N;
	return (tmp);
}
/***************************电感采值************************************
函数：  void ADC_Collect()
功能：  电感采值
参数：  void
说明：  8位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；void
***********************************************************************/
void ADC_Collect()
{
	adc_value[0] = adc_ave(Left_ADC_Pin, ADC_8BIT, 3);		 // 左横电感
	adc_value[1] = adc_ave(LeftXie_ADC_Pin, ADC_8BIT, 3);	 // 左竖电感
	adc_value[2] = adc_ave(RightXie_ADC_Pin, ADC_8BIT, 3);	 // 右竖电感
	adc_value[3] = adc_ave(Right_ADC_Pin, ADC_8BIT, 3);		 // 右横电感
	adc_valueM = adc_ave(Mid_ADC_Pin, ADC_8BIT, 3) * 0.2246; // 电源电压采集
}
/*********************************电感采值********************************
函数：  void Data_current_analyze()
功能：  电感采值原始值归一化（0~100）
参数：  void
说明：  归一化处理
返回值；void
*************************************************************************/
void Data_current_analyze()
{
	uint8 i;
	for (i = 0; i < NM; i++)
	{
		AD_V[i] = ((adc_value[i] - adc_min[i]) * 100) / (adc_max[i] - adc_min[i]);
		if (AD_V[i] <= 0)
		{
			AD_V[i] = 0;
		}
		else if (AD_V[i] >= 100)
		{
			AD_V[i] = 100;
		}
	}
	Left_Adc = AD_V[0];		 // 左电感最终值
	Left_Shu_Adc = AD_V[1];	 // 左竖电感最终值
	Right_Shu_Adc = AD_V[2]; // 右竖电感最终值
	Right_Adc = AD_V[3];	 // 右电感最终值
}

/*********************************差比和函数**********************************
函数：  float Cha_bi_he(int16 data1, int16 data2,int16 x)
功能：  差比和求赛道偏差
参数：  int16 data1, int16 data2,int16 x
说明：  差比和求赛道偏差
返回值；result
****************************************************************************/
float Cha_bi_he(int16 data1, int16 data2, int16 x)
{
	float cha;
	float he;
	float result;

	cha = (data1) - (data2);
	he = data1 + data2 + 1;
	result = (cha * x) / (1.0 * he);

	return result;
}
// 差比和差加权，变式
float Cha_bi_he_cha(int16 data1, int16 data2, int16 data3, int16 data4, int16 x, int16 y)
{
	float cha;
	float he;
	float cha1;
	float he1;

	float result;

	cha = (data1) - (data2);
	cha1 = (data3) - (data4);

	he = data1 + data2 + 1;
	he1 = data3 + data4 + 1;

	//    result = (cha*x)/(1.0*he);
	result = ((cha * x) + (cha1 * y)) / ((1.0 * he) + (1.0 * he1));
	return result;
}
// 向量差比和
float Cha_x_bi_he(int16 data1, int16 data2, int16 data3, int16 data4)
{
	float left_value;
	float right_value;
	float ad_sum;
	float ad_diff;
	float error_x;
	left_value = sqrt(data1 * data1 + data2 * data2);

	right_value = sqrt(data3 * data3 + data4 * data4);

	ad_sum = left_value + right_value + 1; // 计算电感之和

	// 计算电感之差

	ad_diff = (int16)right_value - left_value;
	error_x = ad_diff / ad_sum;

	return error_x;
}
/*****************************************出界保护函数*************************************
函数：  void Out_protect()
参数：  无
说明：  防止车冲出赛道后撞坏东西,检测出赛道后中断失能，电机停转，放回赛道中断使能继续跑

*注意：！！！平时调试时可以打开，加了避障处理后需要关闭此函数，不然有可能无法实现避障功能！！！
返回值：无
******************************************************************************************/
void Out_protect(void)
{
	if (road_type.annulus || road_type.in_annulus_left || road_type.in_annulus_right || road_type.on_annulus_left || road_type.on_annulus_right || road_type.out_annulus)
	{
		if (Left_Adc < OUTSIDE || Right_Adc < OUTSIDE)
		{
			while (1)
			{
				go_motor(-2000, -2000);
				delay_ms(400);
				while (1)
				{
					go_motor(0, 0);
					pwm_duty(PWMB_CH4_P77, 500);
					pwm_duty(PWMB_CH3_P33, 500);
				}
			}
		}
	}
}

void Annulus_assist(void)
{
	if (road_type.annulus == 1 && road_type.in_annulus_right == 0 && road_type.on_annulus_right == 0 && road_type.out_annulus == 0) //&&road_type.in_annulus_left==0
	{
		annulus_s += fabs(last_speed) * 1;
	}
	if (road_type.in_annulus_right == 1) // road_type.in_annulus_left==1 ||                 && road_type.on_annulus_left==0���⣩&& road_type.on_annulus_right==0
	{
		annulus_z += fabs(GORY_Z);
		annulus_s2 += fabs(last_speed) * 1; // ���ݻ��־���ͱ�������ֵ���ģ�0.1��
	}
	if (road_type.on_annulus_right == 1) // road_type.in_annulus_left==1 ||                 && road_type.on_annulus_left==0���⣩&& road_type.on_annulus_right==0
	{
		//        annulus_z += fabs(GORY_Z);
		annulus_s3 += fabs(last_speed) * 1; // ���ݻ��־���ͱ�������ֵ���ģ�0.1��
	}
	if (road_type.out_annulus == 1)
	{
		annulus_t = fabs(last_speed) * 1;
		//		      annulus_t=annulus_t+5;
	}
}

int8 testflag;
/*************************************������������*************************************
函数：  void Annulus_handle(void)
参数：  无
说明：  环岛处理函数

*注意：用两个竖电感引导进环
返回值：无
******************************************************************************************/
void Annulus_handle(void)
{
	if ((Left_Adc + Right_Adc) > IN_ANNULUS_H_LIMIT && road_type.annulus == 0)
	{
		testflag = 1;
		road_type.annulus = 1;
		BUZZ_ON;
	}
	//	  //左环进环判断
	//		if(annulus_s > DISTANCE_ANNULUS_S
	//			 &&(Left_Shu_Adc>20)
	//			 &&road_type.annulus==1
	//			 &&road_type.in_annulus_left==0
	//			 &&road_type.in_annulus_right==0
	//			 &&road_type.on_annulus_left==0
	//			 &&road_type.on_annulus_right==0
	//			 &&road_type.out_annulus==0)
	//		{
	//			road_type.in_annulus_left = 1;
	//			P52                      = 0;
	//			testflag = 2;
	//		}
	// 右环进环判断
	if (annulus_s > DISTANCE_ANNULUS_S
		//		&& (Right_Shu_Adc > 20)
		&& road_type.annulus == 1 && road_type.in_annulus_left == 0 && road_type.in_annulus_right == 0 && road_type.on_annulus_left == 0 && road_type.on_annulus_right == 0 && road_type.out_annulus == 0)
	{
		testflag = 3;
		road_type.in_annulus_right = 1;
		BUZZ_ON;
	}
	//		//左环上环处理
	//		if(annulus_s2 > 40
	////			 &&annulus_z > DISTANCE_ANNULUS_Z
	//			 &&road_type.annulus==1
	//			 &&road_type.in_annulus_left==1
	//			 &&road_type.in_annulus_right==0
	//			 &&road_type.on_annulus_left==0
	//			 &&road_type.on_annulus_right==0
	//			 &&road_type.out_annulus==0)
	//		{
	//				testflag = 4;
	//				road_type.in_annulus_left =0;
	//				road_type.on_annulus_left = 1;
	//				BUZZ_ON;
	//			  P52                      = 1;
	//		}
	// 右环上环处理
	if (annulus_s2 > 1300 && road_type.annulus == 1 && road_type.in_annulus_left == 0 && road_type.in_annulus_right == 1 && road_type.on_annulus_left == 0 && road_type.on_annulus_right == 0 && road_type.out_annulus == 0)
	{
		testflag = 5;
		road_type.in_annulus_right = 0;
		road_type.on_annulus_right = 1;
		BUZZ_ON;
	}
	if (((road_type.on_annulus_right == 1) 
	|| (road_type.on_annulus_left == 1)) 
	&& Left_Adc + Right_Adc > OUT_ANNULUS_S_LIMIT 
	&& annulus_s3 > 200)
	{
		testflag = 6;
		road_type.on_annulus_right = 0;
		road_type.on_annulus_left = 0;
		road_type.out_annulus = 1;
		annulus_s = 0;
		annulus_z = 0;
		annulus_s2 = 0;
		BUZZ_ON;
	}
	// 出环处理
	if (annulus_t > DISTANCE_ANNULUS_T 
		&& road_type.out_annulus == 1)
	{
		testflag = 7;
		road_type.out_annulus = 0;
		annulus_t = 0;
		BUZZ_OFF;
		//					while(1)
		//					{
		//						go_motor(0,0);
		//					}
	}

	// if (annulus_t > DISTANCE_ANNULUS_T)
	// {
	// 	road_type.annulus = 0;
	// 	road_type.in_annulus_left = 0;
	// 	road_type.in_annulus_right = 0;
	// 	road_type.on_annulus_left = 0;
	// 	road_type.on_annulus_right = 0;
	// 	road_type.out_annulus = 0;
	// 	annulus_t = 0;

	// 	BUZZ_OFF;
	// }
}
/*************************根据赛道类型选择不同的方向偏差计算方法*************************
函数：  int16 Direction_error(void)
功能：  根据赛道类型选择不同的方向偏差
参数：  无
说明：  根据赛道类型选择不同的方向偏差
返回值：error--返回赛道偏差
****************************************************************************************/
float Direction_error(void)
{
	float error = 0;

	// ��������ƫ�����
	if (road_type.annulus == 1)
	{
		// ׼���뻷������ƫ�����
		//        if(road_type.annulus==1&&road_type.in_annulus_left==0 && road_type.in_annulus_right==0 && road_type.on_annulus_left==0 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
		//				{
		//					error = Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc)*20;
		//        //���󻷵�����ƫ�����
		//        if(road_type.in_annulus_left ==1 && road_type.on_annulus_left==0 )
		//				{
		//				    //error = Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc)*20;
		//					  error = -15;
		//					road_type.annulus=0;//��������־λ����
		//				}
		// ���һ�������ƫ�����
		if (road_type.in_annulus_right == 1 )
		{
			// error = Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc)*20;					 // error = 3;
			error = 0.5; // �߼��෴��ѭ��

			//				while(1)//��ǽڵ�2
			//					{
			//						go_motor(0,0);
			//					}
		}
		// �ڻ���ƫ��
		if (road_type.on_annulus_right == 1)
		{
			road_type.annulus = 0; // �뻷����־λ����
			road_type.in_annulus_right = 0;
			//					error = Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc)*20;
			error = (Cha_bi_he(Right_Adc, Left_Adc, 20));

			//				while(1)//����ѭ���ڵ�4
			//					{
			//						go_motor(0,0);
			//					}
		}
		// ����������ƫ�����
		if (road_type.out_annulus == 1 && road_type.on_annulus_right == 1)
		{
			//				    error = Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc)*7;
			error = -3;
			//					error = Cha_bi_he(Right_Adc,Left_Adc,5);
			road_type.annulus = 0; // ��������־λ����
								   //				while(1)//�����ڵ�6
								   //					{
								   //						go_motor(0,0);
								   //					}
		}
	}
	// ��������

	else
	{

		//		aim_speed = ZxjsWdjs(Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc),400)+100;
		error = Cha_x_bi_he(Left_Adc, Left_Shu_Adc * 2, Right_Adc, Right_Shu_Adc * 2) * 20; // ��Ļ��ʾ������ƫ��ֵ

		//			errorh=1.0f/exp(errors*errors);												//���ٺ���
		//			aim_speedb=aim_speed*errorh;												//��̬�����ٶȣ�ֱ����˥�����˥����
								//			if(Left_Adc==0&&Left_Shu_Adc==0&&Right_Adc==0&&Right_Shu_Adc==0)
								//			{
								//			aim_speedb = -10;															//���ߵ���
								//			}

		//	}
	}
	return error;
}

/**********************************电磁所有总处理***************************************
函数：  void Get_deviation(void)
功能：  电磁所有总处理
参数：  无
说明：  放中断调用此函数即可
返回值：无
****************************************************************************************/
void Get_deviation(void)
{

	ADC_Collect();					 
	Data_current_analyze();			 
	Annulus_handle();				 
	Annulus_assist();				 
	Current_Dir = Direction_error(); 
}
