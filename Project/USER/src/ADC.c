#include "ADC.h"
#include "math.h"
// int16 aim_speeda        = 730;  //Ŀ���ٶ�
int16 aim_speedb;		// ����ٶȣ���̬�����ٶȣ�=�����ٶ�*�������ٶ�
int16 aim_speedc = 760; // ת���С�ٶ�
float errorh = 0;
float errors = 0;
float errors1 = 0;

int16 adc_value[4]; // �����вɼ�ֵԭʼֵ    4�����
int16 AD_V[4];		// �����вɼ�ֵ��һ��ֵ�м���� ��������ģ�����ɾ����
// int16 adc_max[4]={90,90,90,95}; //��в�ֵ���ֵ ��Ҫ�Լ��ɼ�
int16 adc_max[4] = {250, 200, 175, 250};				// ��в�ֵ���ֵ ��Ҫ�Լ��ɼ�
int16 adc_min[4] = {1, 1, 1, 1};						// ��в�ֵ��Сֵ  1,4,14,1
uint8 Left_Adc, Right_Adc, Left_Shu_Adc, Right_Shu_Adc; // ���ֵ
float adc_valueM;
int8 NM = 4; // ��и���

// ��������
uint16 annulus_s = 0;  // �������־���
uint16 annulus_s2 = 0; // �������־���2
uint16 annulus_s3 = 0;
uint16 annulus_z = 0; // �����ڻ��ִ��
uint16 annulus_t = 0;

uint16 obstacle_annulus_z1 = 0;
uint16 obstacle_annulus_s1 = 0;
uint16 obstacle_annulus_z2 = 0;
uint16 obstacle_annulus_s2 = 0;
uint16 obstacle_annulus_z3 = 0;
uint16 obstacle_annulus_s3 = 0;
uint8 obstacle_switch_1 = 0;
uint8 obstacle_switch_2 = 0;
uint8 obstacle_switch_3 = 0;
uint8 obstacle_switch_4 = 0;

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
/***��ǰλ��*************/
float Current_Dir = 0;
int16 Set_gyro = 0;
int16 ADC_PWM = 0;
uint8 flag_obstacle = 0;
uint16 obstacle_time = 0;
uint8 temp = 0;
/***************************��вɼ�ͨ����ʼ��****************************
������  void ADC_int(void)
���ܣ�  ��в�ֵ���г�ʼ��
������  void
˵����  ��вɼ���ʼ��
����ֵ����
************************************************************************/
void ADC_int(void)
{
	adc_init(Left_ADC_Pin, ADC_SYSclk_DIV_2);	  // ��ʼ��P0.0ΪADC����
	adc_init(LeftXie_ADC_Pin, ADC_SYSclk_DIV_2);  // ��ʼ��P0.1ΪADC����
	adc_init(RightXie_ADC_Pin, ADC_SYSclk_DIV_2); // ��ʼ��P0.5ΪADC����
	adc_init(Right_ADC_Pin, ADC_SYSclk_DIV_2);	  // ��ʼ��P0.6ΪADC����

	adc_init(Mid_ADC_Pin, ADC_SYSclk_DIV_2); // ��ʼ��P1.5ΪADC����
}

/***************************��ֵ�˲�����*********************************
������uint16 adc_mid(ADCN_enum adcn,ADCCH_enum ch)
���ܣ� 3�ε�в�ֵ������ֵ�˲�
������ adcn        ѡ��ADCͨ��       resolution      �ֱ���
˵���� 8λADC�����0~255��2��8�η�����5v��ѹƽ���ֳ�255�ݣ��ֱ���Ϊ5/255=0.196
����ֵ��k(uint8)�м��Ǹ�ֵ
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

/***************************��ֵ�˲�����****************************
������  uint16 adc_ave(ADCN_enum adcn,ADCCH_enum ch,uint8 N)
���ܣ�  ��ֵ�˲����5�����ֵ��ƽ��ֵ
������  adcn        ѡ��ADCͨ��
˵����  �ú���������ֵ�˲������������ֵ����λ��
����ֵ��tmp
ʾ����  adc_ave(ADC_P10, ADC_8BIT)-->ADCͨ��ΪP-10���ֱ���Ϊ8bit
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
/***************************��в�ֵ************************************
������  void ADC_Collect()
���ܣ�  ��в�ֵ
������  void
˵����  8λADC�����0~255��2��8�η�����5v��ѹƽ���ֳ�255�ݣ��ֱ���Ϊ5/255=0.196
����ֵ��void
***********************************************************************/
void ADC_Collect()
{
	adc_value[0] = adc_ave(Left_ADC_Pin, ADC_8BIT, 3);		 // �����
	adc_value[1] = adc_ave(LeftXie_ADC_Pin, ADC_8BIT, 3);	 // �������
	adc_value[2] = adc_ave(RightXie_ADC_Pin, ADC_8BIT, 3);	 // �������
	adc_value[3] = adc_ave(Right_ADC_Pin, ADC_8BIT, 3);		 // �Һ���
	adc_valueM = adc_ave(Mid_ADC_Pin, ADC_8BIT, 3) * 0.2246; // ��Դ��ѹ�ɼ�
}
/*********************************��в�ֵ********************************
������  void Data_current_analyze()
���ܣ�  ��в�ֵԭʼֵ��һ����0~100��
������  void
˵����  ��һ������
����ֵ��void
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
	Left_Adc = AD_V[0];		 // ��������ֵ
	Left_Shu_Adc = AD_V[1];	 // �����������ֵ
	Right_Shu_Adc = AD_V[2]; // �����������ֵ
	Right_Adc = AD_V[3];	 // �ҵ������ֵ
}

/*********************************��Ⱥͺ���**********************************
������  float Cha_bi_he(int16 data1, int16 data2,int16 x)
���ܣ�  ��Ⱥ�������ƫ��
������  int16 data1, int16 data2,int16 x
˵����  ��Ⱥ�������ƫ��
����ֵ��result
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
// ��ȺͲ�
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
float Cha_x_bi_he(int16 data1, int16 data2, int16 data3, int16 data4) // ������Ⱥ�
{
	float left_value;
	float right_value;
	float ad_sum;
	float ad_diff;
	float error_x;
	left_value = sqrt(data1 * data1 + data2 * data2);

	right_value = sqrt(data3 * data3 + data4 * data4);

	ad_sum = left_value + right_value + 1; // ������֮��

	// ������֮��

	ad_diff = (int16)right_value - left_value;
	error_x = ad_diff / ad_sum;

	return error_x;
}
/*****************************************���籣������*************************************
������  void Out_protect()
������  ��
˵����  ��ֹ�����������ײ������,�����������ж�ʧ�ܣ����ͣת���Ż������ж�ʹ�ܼ�����

*ע�⣺������ƽʱ����ʱ���Դ򿪣����˱��ϴ�������Ҫ�رմ˺�������Ȼ�п����޷�ʵ�ֱ��Ϲ��ܣ�����
����ֵ����
******************************************************************************************/
void Out_protect(void)
{
	if (flag_obstacle == 0)
	{
		if (Left_Adc < OUTSIDE && Right_Adc < OUTSIDE)
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

/*************************************������������*************************************
������  void Annulus_assist(void)
������  ��
˵����  ���������������֣��������֣��������ֵ�

*ע�⣺ ����ֵ��������ʱ��Ĳ�ͬ���ı䣬��Ҫ�Լ������Ƴ�ȥ��������Ļ��ʾ������¼ȥ�޸�
����ֵ����
******************************************************************************************/
void Annulus_assist(void)
{
	if (road_type.annulus == 1 && road_type.in_annulus_right == 0) //&&road_type.in_annulus_left==0
	{
		annulus_s += fabs(last_speed) * 1;
	}
	if (road_type.in_annulus_right == 1) // road_type.in_annulus_left==1 ||                 && road_type.on_annulus_left==0���⣩&& road_type.on_annulus_right==0
	{
		annulus_z += fabs(GORY_Z);
		annulus_s2 += fabs(last_speed) * 1; // ���ݻ��־���ͱ�������ֵ���ģ�0.1��
	}
	if (road_type.on_annulus_right == 1 && road_type.in_annulus_right == 1) // road_type.in_annulus_left==1 ||                 && road_type.on_annulus_left==0���⣩&& road_type.on_annulus_right==0
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

uint8 obstacle_number = 0;
/*************************************���ϼ�⺯��*************************************
������  void obstacle_avoidance(void)
������  ��
˵����  TFO����ģ���⣬ʹ������ģ��IICͨ�ţ��������κ����Ŷ�����ʹ�ã�����Ҫע�ⲻ������
		���á�
*ע�⣺ TOFģ�����ϰ���ԽԶ��ֵԽ��Խ����ֵԽС
����ֵ����
******************************************************************************************/
void obstacle_avoidance(void)
{
	dl1a_get_distance(); // �������
	//	if(dl1a_distance_mm<SET_DLLA_DISTANCE&&flag_obstacle==0&&(fabs(Current_Dir)<3)&&obstacle_number==0) //��������С���趨ֵ��־λ����
	//	if(dl1a_distance_mm<SET_DLLA_DISTANCE)
	if (dl1a_distance_mm < SET_DLLA_DISTANCE && flag_obstacle == 0 && (fabs(Current_Dir) < 3)) // ��������С���趨ֵ��־λ����
	{

		flag_obstacle = 0; // ����Ӧ���ǵ���1��ΪĿǰ20��û�б������Ԫ�����Բ�ʹ��
		//		obstacle_number++;//����ֻ�ж�һ�Σ�����ʱͨ�����뿪��ѡ����ⷽ�����߱��ϣ�ֻ��ֱ���жϱ��ϣ���С����
	}
}
/*************************************�ֶ�P*************************************
������  void obstacle_avoidance(void)
������  ��
˵����  TFO����ģ���⣬ʹ������ģ��IICͨ�ţ��������κ����Ŷ�����ʹ�ã�����Ҫע�ⲻ������
		���á�
*ע�⣺ TOFģ�����ϰ���ԽԶ��ֵԽ��Խ����ֵԽС
����ֵ����
******************************************************************************************/
void subsection_p(void)
{
}
/*************************************���ϸ�������*************************************
������  void Annulus_assist(void)
������  ��
˵����  ���������������֣��������֣��������ֵ�

*ע�⣺ ����ֵ��������ʱ��Ĳ�ͬ���ı䣬��Ҫ�Լ������Ƴ�ȥ��������Ļ��ʾ������¼ȥ�޸�
******************************************************************************************/
void Obstacle_assist(void)
{
	if (flag_obstacle == 1)
	{
		obstacle_annulus_z1 += fabs(GORY_Z) * 0.1;
		obstacle_annulus_s1 += fabs(real_speed) * 0.1;
	}
	if (obstacle_switch_1 == 1 && flag_obstacle == 1)
	{
		obstacle_annulus_z2 += fabs(GORY_Z) * 0.1;
		obstacle_annulus_s2 += fabs(real_speed) * 0.1;
	}
	if (obstacle_switch_2 == 1 && obstacle_switch_1 == 1 && flag_obstacle == 1)
	{
		obstacle_annulus_z3 += fabs(GORY_Z) * 0.1;
		obstacle_annulus_s3 += fabs(real_speed) * 0.1;
	}
}
uint8 Annulus_selection = 0;
/*****************************************��������***************************************
������  void Annulus_handle(void)
������  ��
˵����  ������������

*ע�⣺�������������������
����ֵ����
******************************************************************************************/
void Annulus_handle(void)
{
	if ((Left_Adc + Right_Adc) > IN_ANNULUS_H_LIMIT && road_type.annulus == 0) //&&Annulus_selection==0
	{
		road_type.annulus = 1;
		//			aim_speed        = 40;
		BUZZ_ON;
	}
	// �󻷽����ж�
	//		if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_left==0&&(Left_Shu_Adc>20))
	//		{
	//			road_type.in_annulus_left = 1;
	//			BUZZ_ON;
	//			P52                      = 0;
	//		}
	// �һ������ж�
	if (annulus_s > DISTANCE_ANNULUS_S && road_type.in_annulus_right == 0 && road_type.annulus == 1) //&&(Right_Adc>20)
	{
		road_type.in_annulus_right = 1;
		BUZZ_ON;
		P52 = 0;
		//				while(1)//�뻷�ڵ�1
		//					{
		//						go_motor(0,0);
		//					}
	}
	// �󻷴���
	//		if(road_type.in_annulus_left == 1 && annulus_z > DISTANCE_ANNULUS_Z && annulus_s2>350 &&road_type.on_annulus_left==0)
	//		{
	//
	//			  road_type.on_annulus_left = 1;
	//				BUZZ_ON;
	//			  P52                      = 1;
	//		}
	// �һ�����
	if (road_type.in_annulus_right == 1 && annulus_s2 > 1300) //
	{
		road_type.on_annulus_right = 1;
		BUZZ_ON;
		//			  P52                      = 1;
		//			annulus_s2=0;
		//				while(1)//��ǽڵ�3
		//					{
		//						go_motor(0,0);
		//					}
	}
	if (road_type.on_annulus_right == 1 && annulus_s3 > 200 && (Left_Adc + Right_Adc) > OUT_ANNULUS_S_LIMIT) // +Right_Shu_Adc+Left_Shu_Adc
	{

		BUZZ_ON;
		road_type.out_annulus = 1;
		annulus_s = 0;
		annulus_z = 0;
		annulus_s2 = 0;
		//		annulus_s3                 = 0;
		//		while(1)//�����ڵ�5
		//					{
		//						go_motor(0,0);
		//					}
	}
	// ��������
	if (annulus_s3 > DISTANCE_ANNULUS_T && road_type.out_annulus == 1 && road_type.annulus == 0)
	{
		road_type.annulus = 0;
		road_type.in_annulus_left = 0;
		road_type.in_annulus_right = 0;
		road_type.on_annulus_left = 0;
		road_type.on_annulus_right = 0;
		road_type.out_annulus = 0;
		annulus_t = 0;
		P52 = 1;
		BUZZ_OFF;
		//					while(1)//����7
		//					{
		//						go_motor(0,0);
		//					}
	}
}
/*************************������������ѡ��ͬ�ķ���ƫ����㷽��*************************
������  int16 Direction_error(void)
���ܣ�  ������������ѡ��ͬ�ķ���ƫ��
������  ��
˵����  ������������ѡ��ͬ�ķ���ƫ��
����ֵ��error--��������ƫ��
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
		if (road_type.in_annulus_right == 1 && road_type.on_annulus_right == 0 && road_type.out_annulus == 0)
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

	else if (flag_obstacle == 1 && obstacle_number <= 1)
	{
		P52 = 0;
		error = 6;

		// go_motor(0,2000);
		if (obstacle_annulus_s1 > 400) //
		{

			obstacle_switch_1 = 1;
			error = -8;
			//				while(1)
			//					{
			//					  go_motor(0,0);
			//					}
		}
		if (obstacle_annulus_s2 > 400)
		{
			obstacle_switch_2 = 1;
			error = -3.5;
		}
		if (obstacle_annulus_s3 > 150) // obstacle_annulus_z3>100&&     &&(Right_Adc+Left_Adc>30)
		{
			//					error = 3;

			//					while(1)
			//					{
			//					  go_motor(0,0);   //&&obstacle_annulus_s2>400
			//
			//					}
			obstacle_switch_1 = 0;
			obstacle_switch_2 = 0;
			obstacle_annulus_z1 = 0;
			obstacle_annulus_z2 = 0;
			obstacle_annulus_z3 = 0;
			obstacle_annulus_s1 = 0;
			obstacle_annulus_s2 = 0;
			obstacle_annulus_s3 = 0;
			BUZZ_OFF;

			P52 = 1;
			flag_obstacle = 0; // ���Ͻ�����־λ����
			obstacle_number++; // ����ֻ�ж�һ�Σ�����ʱͨ�����뿪��ѡ����ⷽ�����߱��ϣ�ֻ��ֱ���жϱ��ϣ���С����
			//					while(1)
			//					{
			//						go_motor(0,0);
			//					}
		}
	}
	else
	{

		//		aim_speed = ZxjsWdjs(Cha_x_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc),400)+100;
		error = Cha_x_bi_he(Left_Adc, Left_Shu_Adc * 2, Right_Adc, Right_Shu_Adc * 2) * 20; // ��Ļ��ʾ������ƫ��ֵ
		errors = Cha_x_bi_he(Left_Adc, Left_Shu_Adc, Right_Adc, Right_Shu_Adc);				// ��һ���ı�׼����ƫ��ֵ

		//			errorh=1.0f/exp(errors*errors);												//���ٺ���
		//			aim_speedb=aim_speed*errorh;												//��̬�����ٶȣ�ֱ����˥�����˥����
		aim_speedb = aim_speed; // ��̬�����ٶȣ�ֱ����˥�����˥����
		//			if(Left_Adc==0&&Left_Shu_Adc==0&&Right_Adc==0&&Right_Shu_Adc==0)
		//			{
		//			aim_speedb = -10;															//���ߵ���
		//			}

		//	}
	}
	return error;
}

/**********************************��������ܴ���***************************************
������  void Get_deviation(void)
���ܣ�  ��������ܴ���
������  ��
˵����  ���жϵ��ô˺�������
����ֵ����
****************************************************************************************/
void Get_deviation(void)
{

	ADC_Collect();			// ���ԭʼֵ��ֵ
	Data_current_analyze(); // ���ֵ��һ������
	Annulus_handle();		// ��������
	Annulus_assist();		// ������������
	obstacle_avoidance();	// �ϰ�����
	Obstacle_assist();
	Current_Dir = Direction_error(); // �������ƫ��
}
