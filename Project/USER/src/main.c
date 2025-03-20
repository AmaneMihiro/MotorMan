
#include "headfile.h"
void main()
{
	DisableGlobalIRQ();     																	//�ر����ж�
	board_init();		       																		// ��ʼ���Ĵ���,��ɾ���˾���롣
	delay_ms(800);          																	//�����΢��ʱһ��


//	lcd_init();
	ips114_init();    																				//1.14��Һ������ʼ��

	ADC_int();      																					//ADC�ɼ���ʼ��
	ips114_showstr(0,0,(unsigned char*)"Electromagnetic-Car");//�ַ�����ʾ
	ips114_showstr(0,1,"interlize...");
	delay_ms(500);                     
//	init_Steer_PWM();                                          //�����ʼ��
	init_PWM(MOTOR_MODE_SELECT);                               //��ʼ��DRV������ʽ��1-DRV/0-BTN��
	encoder_init();                                            //��������ʼ��
	
//	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_2);//��ʼ�����ߴ���
//	pwm_init(PWMB_CH4_P77, 50, 750);                         //��ʼ����ˢ���20%ռ�ձ�			
//	pwm_init(PWMB_CH3_P33, 50, 750);                         //(1.2ms/20ms * 10000)��10000��PWM����ռ�ձ�ʱ���ֵ�� 10000ΪPWM���ֵ
	delay_ms(500);                                          //��ʱ����ˢ�����ת����
	gpio_mode(Reed_Switch_Pin,GPI_IMPEDANCE);                  //ͣ��ʶ��ĸɻɹ�IO��ʼ��
	PID_int();                                                 //PID������ʼ��  
	ips114_showstr(0,2,"icm20602_int...");  
	icm20602_init();
	ips114_showstr(0,3,"icm20602_intok...");
	delay_ms(500);
	ips114_showstr(0,4,"dlla_int...");                       
	delay_ms(500);
  
//	dl1a_init();                                               //����ģ���ʼ��
	ips114_showstr(0,5,"dlla_intok...");
	pit_timer_ms(TIM_1, 5);                                    //��ʼ����ʱ��1��Ϊ�����жϴ�������1MS��һ���ж�
	NVIC_SetPriority(TIMER1_IRQn, 3);	                         //���ö�ʱ��1�ж����ȼ�Ϊ 3 ������߼���
	gpio_mode(P6_7,GPO_PP);                                    //��������ʼ��
	BUZZ_DiDiDi(250);                                          //��������һ��
	
	ips114_showstr(0,6,"intall_intok...");
	delay_ms(500);
	ips114_clear(BLUE);
	EnableGlobalIRQ(); //��ʼ����ϣ��������ж�
	
	/****����Ĳ��Ժ���ֻ�ǲ����ã����Խ�����ע�͹رգ�һ��ֻ����һ�����Ժ�������******/
//	Test_Servo_Hardware();//���Զ����ֵ
//	Test_Motor_Hardware();//���Ե��ʹ��
//	Test_Electric_Hardware();//���Ե�ŵ�вɼ�
//	Test_Encoder();//���Ա������ɼ�//         
	/*********************************************************************************/

  while(1)
	{	
		    Roundabout_debugshow();//����������Ļ��ʾ�����Ի���ʱ�򿪣�

//			��Դ��ѹ��ʾ			
			ips114_showfloat(190,0,adc_valueM,2,2);  			
  }
}
