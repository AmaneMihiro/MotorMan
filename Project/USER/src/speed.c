#include "speed.h"
#include "math.h"

int16 aim_speed        = 5;  //目标速度  
int16 real_speed       = 0;    //左右轮平均速度 
int16 left_speed       = 0;    //左轮速度
int16 right_speed      = 0;    //右轮速度
int16 last_speed       = 0;    
int16 left_last_speed  = 0;
int16 right_last_speed = 0;
int16 left_real_speed  = 0;
int16 right_real_speed = 0;

int16 All_PWM_left     = 0;     //左轮所有PWM输出
int16 All_PWM_right    = 0;     //右轮所有PWM输出
int16 Speed_pwm_left   = 0;     //左轮速度环PWM（C车用）
int16 Speed_pwm_right  = 0;     //右轮速度环PWM（C车用）
int16 Real_Speed_left  = 0;     //左轮实际速度
int16 Real_Speed_right = 0;     //右轮实际速度
int16 Speed_pwm_all    = 0;     //左右平均速度环PWM（D车用）
int16 Steer_pwm        = 0;     //转向内环PWM
uint16 Open_pack_time  = 0;     //打开干簧管定时
uint16 Stop_time       = 0;     //停车定时
 
/******************************* 电机初始化***********************************
函数：  void init_PWM(unsigned char Motor_Set)
参数：  Motor_Set---为0时初始化为BTN驱动方式，为1时初始化DRV驱动方式
说明：  分母10000
		    pwm_init(PWMA_CH1P_P60, 10000, 0); 
        初始化PWM  使用引脚P6.0  输出PWM频率10000HZ  占空比为百分之 pwm_duty / PWM_DUTY_MAX * 100				
返回值：无  
*****************************************************************************/
unsigned char MOTOR_MODE=0;//中间变量，请勿修改删除！！！
void init_PWM(unsigned char Motor_Set)
{
	MOTOR_MODE = Motor_Set;
	if (MOTOR_MODE==0) 
	{
//-----MOS驱动-----------
    pwm_init(Left_Z_Pin,  20000,0);//左轮初始化
	  pwm_init(Left_F_Pin,  20000,0);
	  pwm_init(Right_Z_Pin, 20000,0);//右轮初始化
	  pwm_init(Right_F_Pin, 20000,0);
	}
		else
	{
//------DRV驱动-------------
	  pwm_init(Left_PWM_Pin, 20000,0);//左轮初始化
  	pwm_init(Right_PWM_Pin,20000,0);//右轮初始化
		gpio_mode(P6_4,GPO_PP);       // 设置DRV方向引脚为为推挽输出
	  gpio_mode(P6_0,GPO_PP);       // 设置DRV方向引脚为为推挽输出
 } 
}
/****************************编码器初始化****************************
函数：  void encoder_init(void)
功能：  编码器初始化
参数：  无
说明：  ctimer_count_init(CTIM0_P34);
        编码器使用TIM3和TIM4，如更改引脚只需修改宏定义即可 
        编码器使用带方向的编码器（STC不支持正交解码）
返回值：无
********************************************************************/
void encoder_init()
{
    //左编码器初始化
		ctimer_count_init(Left_Ecoder_Pin1);
		//右编码器初始化
		ctimer_count_init(Right_Ecoder_Pin1);
}  
/***************************速度测量********************************
函数名：speed_measure()
功  能：速度测量，读取编码器的值，不同编码器安装和车的前进方向不对会
        导致采集的值可能是反过来的，只需修改* (-1)就行，改到上面就可
        以了
参  数：void
返回值：void
******************************************************************/
void speed_measure()
{ 
	  int16 temp_L, temp_R;
    temp_L = ctimer_count_read(Left_Ecoder_Pin1); // 左右轮当前速度
    temp_R = ctimer_count_read(Right_Ecoder_Pin1);

    ctimer_count_clean(Left_Ecoder_Pin1); // 编码器清零
    ctimer_count_clean(Right_Ecoder_Pin1);
	  
	  if(Left_Ecoder_Pin2 == 1)
        left_speed = temp_L;
    else
        left_speed = (1) * temp_L;
    if(Right_Ecoder_Pin2 == 0)
        right_speed = temp_R;
    else
        right_speed = (1) * temp_R;
		
		last_speed = (right_speed+left_speed)/2;// 速度平均值
	  real_speed*=0.8;            //一阶低通滤波器
	  real_speed+=last_speed*0.2; //一阶低通滤波器  left_last_speed
		
		left_last_speed =left_speed;
	  left_real_speed*=0.8;       //一阶低通滤波器
	  left_real_speed+=left_last_speed*0.2; //一阶低通滤波器
		
		right_last_speed =right_speed;
	  right_real_speed*=0.8;        //一阶低通滤波器
	  right_real_speed+=right_last_speed*0.2; //一阶低通滤波器
}
/*******************************出库定时打开干簧管等***********************************
函数：  void timed_task(void)
参数：  无
说明：  出库完成定时打开干簧管等作为标志位处理，防止刚出库就检测到停车
返回值：无 
*************************************************************************************/
void timed_task(void)
{
	if(flag_start)
	{
		Open_pack_time=Open_pack_time+10;
	}
	if(flag_end)
	{
		T_inku_wait=T_inku_wait+5;
	}
	if(T_inku_J)
	{
		T_inku_S=T_inku_S+5;
	}
}

/*****************************电机输出*******************************************
函数：void go_motor (int16 left_PWM,int16 right_PWM)
参数：  int16 left_PWM,int16 right_PWM
说明：pwm_duty(PWMA_CH1P_P60, duty);
      务必将车子的电机逻辑调像这个一样，第一个参数控制左电机，第二个参数控制右电机
      传入的参数如果为正数，电机正转，负值反转！！！！！
返回值：无 
********************************************************************************/
#define Duty_Max  6000   //限幅最大值

void go_motor (int16 left_PWM,int16 right_PWM)
{
  if  (MOTOR_MODE==0)
  {	
//---------------------------------MOS驱动-----------------------------------------	
    if (left_PWM>0)                   //左轮
    {
		 left_PWM = left_PWM<=Duty_Max ? left_PWM : Duty_Max;
     pwm_duty(Left_Z_Pin,left_PWM);
     pwm_duty(Left_F_Pin,0);			    //正转
    } 	
    else 
    {
     left_PWM = left_PWM>=-Duty_Max ? (-left_PWM) : Duty_Max;  
     pwm_duty(Left_Z_Pin,1);	
     pwm_duty(Left_F_Pin,left_PWM);	  //反转
    } 
    if (right_PWM>0)                  //右轮
    { 
     right_PWM = right_PWM<=Duty_Max ? right_PWM : Duty_Max; 
	   pwm_duty(Right_Z_Pin,right_PWM);	
	   pwm_duty(Right_F_Pin,0);			    //正转
    } 
    else 
    {
     right_PWM = right_PWM>=-Duty_Max ? (-right_PWM) : Duty_Max;  
	   pwm_duty(Right_Z_Pin,1);	
	   pwm_duty(Right_F_Pin,right_PWM); //反转
    }
  }
  else
  {
//-------------------------------------------DRV驱动-------------------------------------
   if (left_PWM>0)                     //左轮
   {
		 left_PWM = left_PWM<=Duty_Max ? left_PWM : Duty_Max;
		 Left_DIR_Pin=0;			 
     pwm_duty(Left_PWM_Pin,left_PWM);  //正转
   } 	
   else 
   {
     left_PWM = left_PWM>=-Duty_Max ? (-left_PWM) : Duty_Max;  
     Left_DIR_Pin=1;	
     pwm_duty(Left_PWM_Pin,left_PWM);  //反转
   }
   if (right_PWM>0)                    //右轮
   {
     right_PWM = right_PWM<=Duty_Max ? right_PWM : Duty_Max;
     Right_DIR_Pin=0;			 
	   pwm_duty(Right_PWM_Pin,right_PWM);//正转		
	 } 
   else 
   {
     right_PWM = right_PWM>=-Duty_Max ? (-right_PWM) : Duty_Max;  
	   Right_DIR_Pin=1;
	   pwm_duty(Right_PWM_Pin,right_PWM); //反转
   }
  }
}