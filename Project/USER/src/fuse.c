#include "fuse.h"
#include "math.h"

PID SpeedPID = {0};
PID L_SpeedPID ={0};
PID R_SpeedPID ={0};
PID TurnPID ={0};

int16 GORY_Z=0;
/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
返回值：void
********************************************************************************/
void PID_int(void)
{
//	SpeedPID.Kp=50;     //0.6//速度环PID参数（D车用，速度环2ms）18
//	SpeedPID.Ki=10 ;     //0.5                                  2.5
//	SpeedPID.Kd=0;
	
	L_SpeedPID.Kp=330; //4  //左轮速度环PID参数（速度环20ms）（6）4.5
	L_SpeedPID.Ki=0.7;//1
	L_SpeedPID.Kd=0;
	
	R_SpeedPID.Kp=330;   //右速度环PID参数（速度环20ms）（6）5.5
	R_SpeedPID.Ki=0.7;
	R_SpeedPID.Kd=0;
	
//	TurnPID.Kp=0;       //转向环PID参数 
//	TurnPID.Ki=0; //0.08
//	TurnPID.Kd=0;
//	
//	TurnPID.Kp=4;       //转向环PID参数 
//	TurnPID.Ki=0; //0.08
//	TurnPID.Kd=3;
//	
	TurnPID.Kp=1.3;       //转向环PID参数 
	TurnPID.Ki=0; //0.08
	TurnPID.Kd=4;
	
	TurnPID.K_gory=0;
}
//void PID_int(void)
//{
//	SpeedPID.Kp=18;     //0.6//速度环PID参数（D车用，速度环2ms）18
//	SpeedPID.Ki=3 ;     //0.5                                  2.5
//	SpeedPID.Kd=0;
//	
//	L_SpeedPID.Kp=4; //4  //左轮速度环PID参数（速度环20ms）（6）4.5
//	L_SpeedPID.Ki=0.15;//1
//	L_SpeedPID.Kd=0;
//	
//	R_SpeedPID.Kp=4;   //右速度环PID参数（速度环20ms）（6）5.5
//	R_SpeedPID.Ki=0.15;
//	R_SpeedPID.Kd=0;
//	
//	TurnPID.Kp=87;       //转向环PID参数 
//	TurnPID.Ki=0; //0.08
//	TurnPID.Kd=0.4;
//	TurnPID.K_gory=3.5;
//}
// 别名
static TASK_COMPONENTS TaskComps[] =
{
    {0,  1,  1, Motor_output_control},         //角速度内环和D车速度环2ms
//    {0, 2, 2, Trailing_control},           //转向外环10ms
//    {0, 4, 4, Speed_control},              //C车速度环20ms
};
/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理
* EntryParameter : None
* ReturnValue    : None
* attention      : ***在定时器中断中调用此函数即可***
**************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;
    for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
    {
        if (TaskComps[i].Timer)          // 时间不为0
        {
           TaskComps[i].Timer--;         // 减去一个节拍
           if (TaskComps[i].Timer == 0)       // 时间减完了
           {
             TaskComps[i].Timer = TaskComps[i].ItvTime; // 恢复计时器值，从新下一次
             TaskComps[i].Run = 1;           // 任务可以运行
           }
        }
   }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理|判断什么时候该执行那一个任务
* EntryParameter : None
* ReturnValue    : None
* * attention      : ***放在mian的while(1)即可***
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i; 
	  for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
    {
        if (TaskComps[i].Run)           // 时间不为0
        {
            TaskComps[i].TaskHook();       // 运行任务
            TaskComps[i].Run = 0;          // 标志清0
        }
    }
}
/****************************角速度内环和D车速度环**************************************
函数：  void Motor_output_control()
参数：  void
说明：  角速度内环和D车速度环(D车/三轮车才会用)
返回值：void
***************************************************************************************/
void Motor_output_control()
{
//	P52=0;
	
	  //imu660ra_get_gyro();   //获取660陀螺仪角速度值
	  icm20602_get_gyro();  
	  GORY_Z=icm20602_gyro_transition (icm20602_gyro_z);
	  //GORY_Z= imu660ra_gyro_transition(imu660ra_gyro_z);         // 单位为°/s
	  speed_measure();       //编码器测量
	  Get_deviation();       //电磁采集并获取赛道偏差

	
//	  timed_task();          //定时操作
	
	  ADC_PWM = PID_Turn_DT(&TurnPID,Current_Dir,GORY_Z);//动态位置式PID  left_real_speed
//	  ADC_PWM=range_protect(ADC_PWM, -500, 500);
//	  Speed_pwm_all  += IncPIDCalc(&SpeedPID,aim_speed,real_speed); 
//    Real_Speed_left+=fabs(real_speed)*0.1;
//	  if(Real_Speed_left>800)
//		{
//			aim_speed=0;
//			Out_protect();         //出界保护
//		}
	 

	  Speed_pwm_left  += IncPIDCalc(&L_SpeedPID,aim_speedb+ADC_PWM,left_real_speed);  
	  Speed_pwm_right += IncPIDCalc(&R_SpeedPID,aim_speedb-ADC_PWM,right_real_speed); 
	  
//	  Speed_pwm_left=range_protect(Speed_pwm_left, -aim_speed, 2*aim_speed);//减速限幅（防止轮子反转太大导致假期望速度）
//	  Speed_pwm_right=range_protect(Speed_pwm_right, -aim_speed,2*aim_speed);//减速限幅（防止轮子反转太大导致假期望速度）
//	  if(Speed_pwm_left<-aim_speed)
//	  {
//	  Speed_pwm_left=-aim_speed;
//	  }
//	  if(Speed_pwm_right<-aim_speed)
//	  {
//	  Speed_pwm_right=-aim_speed;
//	  }
//	  All_PWM_left  = Speed_pwm_all+ADC_PWM; 
//	  All_PWM_right = Speed_pwm_all-ADC_PWM;
	  
	  go_motor(Speed_pwm_left,Speed_pwm_right);                         //动力输出

//	  pwm_duty(PWMB_CH4_P77, 800);
//		pwm_duty(PWMB_CH3_P33, 800);
	   //go_motor(2000,2000);  
//		Out_protect();         //出界保护
//P52=1;
}
/****************************转向环（D车转向外环）**************************************
函数：  void Trailing_control()
参数：  void
说明：  转向环（D车转向外环）（C车转向环）
返回值：void
***************************************************************************************/
void Trailing_control()
{
//	Get_deviation();  //电磁采集并获取赛道偏差
//	speed_measure();       //编码器测量
//	
//	ADC_PWM = LocP_DCalc(&TurnPID,Current_Dir,0);//位置式PD控制转向
////	Out_protect();         //出界保护
//	ADC_PWM = PlacePID_Control(&TurnPID,Current_Dir,0);//动态位置式PID控制 
//	ADC_PWM = PlacePID_Control(&TurnPID,Current_Dir,0);//动态位置式PID控制 (用于转向控制)
	  //Steering_Control_Out(ADC_PWM);//(C车用控制舵机转向)
}
/****************************速度环（C车用）**************************************
函数：  void Speed_control()
参数：  void
说明：  速度环（C车用）
***************************************************************************************/
void Speed_control()
{
	  //timed_task();           //出库定时打开干簧管等
	  //speed_measure();      //编码器测量
	 // aim_speed = 450;      //目标速度
	
	  //Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed ,real_speed); //D车速度环（位置式）
	  //Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);//D车速度环（增量式）
	  
    //Speed_pwm_left += IncPIDCalc(&L_SpeedPID,aim_speed , left_speed); //C车左轮速度环（位置式）
	  //Speed_pwm_right += IncPIDCalc(&R_SpeedPID, aim_speed, right_speed); //C车右轮速度环（位置式）
	  //go_motor(Speed_pwm_left,Speed_pwm_right);                         //动力输出
}
/***************************************************************************************
函数名：int16 range_protect(int16 duty, int16 min, int16 max)
功  能：限幅保护 
参  数：
返回值：duty
**************************************************************************************/
int16 range_protect(int16 duty, int16 min, int16 max)//限幅保护
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}
