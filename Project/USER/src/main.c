
#include "headfile.h"

void main()
{
	DisableGlobalIRQ();     																	//关闭总中断
	board_init();		       																		// 初始化寄存器,勿删除此句代码。
	delay_ms(800);          																	//软件稍微延时一下

	ips114_init();    																				//1.14寸液晶屏初始化

	ADC_int();      																					//ADC采集初始化
	ips114_showstr(0,0,(unsigned char*)"Electromagnetic-Car");//字符串显示
	ips114_showstr(0,1,"interlize...");
	delay_ms(500);                     
	init_PWM(MOTOR_MODE_SELECT);                               //初始化DRV驱动方式（1-DRV/0-BTN）
	encoder_init();                                            //编码器初始化
	
//	uart_init(UART_4, UART4_RX_P02, UART4_TX_P03, 115200, TIM_2);//初始化无线串口
//	pwm_init(PWMB_CH4_P77, 50, 750);                         //初始化无刷电机20%占空比			
//	pwm_init(PWMB_CH3_P33, 50, 750);                         //(1.2ms/20ms * 10000)（10000是PWM的满占空比时候的值） 10000为PWM最大值
	delay_ms(500);                                          //延时让无刷电机先转起来
	gpio_mode(Reed_Switch_Pin,GPI_IMPEDANCE);                  //停车识别的干簧管IO初始化
	PID_int();                                                 //PID参数初始化  
	ips114_showstr(0,2,"icm20602_int...");  
	icm20602_init();
	ips114_showstr(0,3,"icm20602_intok...");
	delay_ms(500);

	pit_timer_ms(TIM_1, 5);                                    //初始化定时器1作为周期中断触发器，1MS进一次中断
	NVIC_SetPriority(TIMER1_IRQn, 3);	                         //设置定时器1中断优先级为 3 级（最高级）
	gpio_mode(P6_7,GPO_PP);                                    //蜂鸣器初始化
	BUZZ_DiDiDi(250);                                          //蜂鸣器滴一声
	
	ips114_showstr(0,4,"intall_intok...");
	delay_ms(500);
	ips114_clear(BLUE);
//	EnableGlobalIRQ(); //初始化完毕，开启总中断
	
	/****下面的测试函数只是测试用，测试结束请注释关闭，一次只允许开一个测试函数！！******/
//	Test_Motor_Hardware();//调试电机使用
//	Test_Electric_Hardware();//测试电磁电感采集
//	Test_Encoder();//测试编码器采集//         
	/*********************************************************************************/

	while (1)
	{

	  
    Roundabout_debugshow();//环岛调试屏幕显示（调试环岛时打开）
//		 Speed_debugshow();     //速度环调试屏幕显示（调试速度环时打开，速度环建议用上位机看波形）
//		  datasend();          //上位机发送数据 

		// ips114_showint16(50, 0, left_real_speed);
		// ips114_showint16(50, 1, right_real_speed);
		// ips114_showint16(50, 2, real_speed);
		// ips114_showint16(50, 4, aim_speed);


		// ips114_showint16(0, 0, Left_Adc); // road_type.straight
		// ips114_showint16(0, 1, Left_Shu_Adc);
		// ips114_showint16(0, 2, Right_Shu_Adc);
		// ips114_showint16(0, 3, Right_Adc);
//		ips114_showuint16(100, 5, dl1a_distance_mm);
//		ips114_showuint16(100, 1, );
////			赛道偏差显示
		// ips114_showfloat(0, 5, Current_Dir, 2, 1);        

//			电源电压显示	
//		ips114_showstr(140, 0, "Value:");
		ips114_showfloat(190, 0, adc_valueM, 2, 2);
	}
}
