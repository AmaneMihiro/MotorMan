#include "park.h"
#include "math.h"

int8 flag_start = 0;	 // ������ɱ�־λ
int8 flag_open_reed = 0; // �򿪸ɻɹ�
int8 reed_state = 0;	 // �ɻɹ�״̬
int8 flag_end = 0;		 // ��ʼͣ����־λ
uint16 T_outku = 0;		 // ����ֱ�ߺʹ�Ƕ�ʱ
uint16 J_outku = 0;
uint16 T_inku_wait = 0;
uint16 T_inku_J = 0;
uint16 T_inku_S = 0;
uint16 T_inku = 0;

uint16 S_daoku = 0;	  //
uint16 S_daoku_2 = 0; //
uint16 S_daoku_3 = 0; //

uint8 Library_selection = 1;

/*****************************************���⺯��***************************************
������  void Handle_Barn_Out(uint8 type)
������  type-----1Ϊ����⣬2Ϊ�ҳ���
˵����  ���⺯��

*ע�⣺���ô˺�����ִ�г��������ֱ�ߵ�ʱ��ʹ��ʱ�估ռ�ձȰ����Լ��޸ĵ���
����ֵ����
******************************************************************************************/
void Handle_Barn_Out(uint8 type)
{
	// 1Ϊ����⣬2Ϊ�ҳ���
	if (type == 1)
	{
		if (!flag_start)
		{
			// pwm_duty(PWMB_CH1_P74,STEER_MID);              //��C���ã�
			go_motor(1500, 1500);		// ������ص�ѹ��ת����Ӱ�죬����������ĵ�ص�
			if (T_outku >= T_OUT_PARK1) // T_OUT_PARK1----����ֱ�ߵ�ʱ��
			{
				// pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM);	//��C���ã�
				go_motor(0, 2000); // ����
				// ips114_showuint16(0,1,J_outku);
			}
			if (T_outku > T_OUT_PARK2) // T_OUT_PARK2----������ʱ�䣨���ʱ��ΪT_OUT_PARK2-T_OUT_PARK1��
			{
				flag_start = 1;
				T_outku = 0;
			}
		}
	}
	if (type == 2)
	{
		if (!flag_start)
		{
			// pwm_duty(PWMB_CH1_P74,STEER_MID);            //��C���ã�
			go_motor(1500, 1500);
			if (T_outku >= T_OUT_PARK1)
			{
				// pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM); //��C���ã�
				go_motor(2000, 0); // �Ҵ��
			}
			if (T_outku > T_OUT_PARK2)
			{
				flag_start = 1;
				T_outku = 0;
			}
		}
	}
}
/*****************************************�ɻɹܼ��ͣ��***************************************
������  void Reed(void)
������  void
˵����  �ɻɹܼ��ͣ��

*ע�⣺ �ɻɹ�ʹ�÷����ͺͰ������ƣ�ͨ����ȡIO�ڵ�ƽ����
����ֵ��void
*********************************************************************************************/
void Reed(void)
{
	if (flag_start) // ��ʼʱ�������ɻɹܼ�⣬��ֹ����ʱ���
	{
		// �߹�һ�ξ�������ɻɹܼ��
		if (Open_pack_time > START_T)
		{
			flag_open_reed = 1;
			Open_pack_time = 0;
		}
	}
	if (flag_open_reed == 0) // �ɻɹܼ���־λ������ſ�ʼ���
	{
		reed_state = Reed_Switch_Pin; // �ɻɹ�״̬
		if (reed_state == 0)
		{
			flag_end += 1; // ʶ��ͣ����־λ����
		}
	}
}
/*****************************************��⺯��***************************************
������  void Reed(void)
������  void
˵����  ��⺯��


����ֵ��void
*********************************************************************************************/
void In_park(uint8 type)
{
	if (type == 1)
	{
		if (flag_end == 1)
		{
			go_motor(0, 0);
			aim_speed = 0;
			while (1)
			{
				speed_measure(); // ����������
				S_daoku += fabs(real_speed) * 0.1;
				go_motor(-1500, -1500);
				ips114_showint16(0, 3, S_daoku);
				BUZZ_ON;
				while (S_daoku > 450)
				{
					speed_measure(); // ����������
					go_motor(1500, 1500);
					S_daoku_2 += fabs(real_speed) * 0.1;
					ips114_showint16(0, 4, S_daoku_2);
					while (S_daoku_2 > 100)
					{
						speed_measure(); // ����������
						go_motor(0, 2000);
						S_daoku_3 += fabs(real_speed) * 0.1;
						ips114_showint16(0, 5, S_daoku_3);
						while (S_daoku_3 > 150)
						{
							go_motor(0, 0);
							BUZZ_OFF;
						}
					}
				}
			}
		}
	}
	if (type == 2)
	{
		if (flag_end == 1)
		{
			go_motor(0, 0);
			aim_speed = 0;
			while (1)
			{
				speed_measure(); // ����������
				S_daoku += fabs(real_speed) * 0.1;
				go_motor(-1500, -1500);
				ips114_showint16(0, 3, S_daoku);
				BUZZ_ON;
				while (S_daoku > 450)
				{
					speed_measure(); // ����������
					go_motor(1500, 1500);
					S_daoku_2 += fabs(real_speed) * 0.1;
					ips114_showint16(0, 4, S_daoku_2);
					while (S_daoku_2 > 100)
					{
						speed_measure(); // ����������
						go_motor(2000, 0);
						S_daoku_3 += fabs(real_speed) * 0.1;
						ips114_showint16(0, 5, S_daoku_3);
						while (S_daoku_3 > 150)
						{
							go_motor(0, 0);
							BUZZ_OFF;
						}
					}
				}
			}
		}
	}
}
