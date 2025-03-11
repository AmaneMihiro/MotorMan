#include "PID.h"
#include "math.h"
/************************************************
函数名：IncPIDInit(PID *sptr)
功  能：PID参数初始化
参  数：
返回值：void
************************************************/
void IncPIDInit(PID *sptr)
{
    sptr->SumError=0;
    sptr->LastError=0;
    sptr->LLastError=0;

    sptr->Kp=0;
    sptr->Ki=0;
    sptr->Kd=0;
}

/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：位置式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float 
************************************************/
int16 LocP_DCalc(PID*sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,dError;
    int16 output;

    iError=Setpoint-Turepoint;  //偏差
    sptr->SumError+=iError;            //积分(采样时间很短时，用一阶差分代替一阶微分，用累加代替积分)
    dError=(int16)(iError-(sptr->LastError));     //微分
    sptr->LastError=iError;
    if(sptr->SumError>2000) sptr->SumError=2000;   //积分限幅
    if(sptr->SumError<-2000) sptr->SumError=-2000;
    output=(int16)(sptr->Kp*iError  //比例项
          +(sptr->Ki*sptr->SumError)//积分项
          +sptr->Kd*dError);        //微分项
    return(output);
}
/************************************************
函数名：IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：增量式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 iIncpid
************************************************/
int16 IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,iIncpid;
    //当前误差
    iError=Setpoint-Turepoint;      //偏差
     
    iIncpid=sptr->Kp*(iError-sptr->LastError)
             +sptr->Ki*iError;
//    //储存误差，用于下次计算
//    if(iIncpid>=100)   //每次输出增量限幅
//		{			
//			iIncpid=100;
//		}
//	  if(iIncpid<=-100)  
//		{			
//			iIncpid=-100;
//		}
			sptr->LastError=iError;
      return(iIncpid);
}
/************************************************
函数名：PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
功  能：动态位置式PID控制 (一般用于转向控制)
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 Actual
************************************************/
int16 PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
{
    int16 iError,Actual;
    float KP;  //动态P，注意与Kp区分

    iError = Setpoint - Turepiont;
    KP = (iError)*sptr->Ki +sptr->Kp;    //动态P的计算
    //sptr->SumError+=iError;

    Actual =KP * iError
            + sptr->Kd* (iError - sptr->LastError);

    sptr->LastError = iError;
    return Actual;
}
/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：D车位置式PID控制，加了陀螺仪
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float 
************************************************/
int16 PID_Turn_DT(PID*sptr,int16 Error,int16 Gory_z)
{
    int16 iError,dError,gory_z;
	  float KP;            //动态P，注意与Kp区分
    int16 output;

    iError=Error;  //偏差
	  gory_z=Gory_z;
	
	  KP = (iError*iError)*sptr->Ki + sptr->Kp;    //动态P的计算
	
    dError=(iError-(sptr->LastError));     //微分
	
    output=KP*iError               //比例项(动态p)
          +sptr->Kd*dError         //微分项
	        +sptr->K_gory*gory_z;    //陀螺仪修正
	
	  sptr->LastError=iError;
	
    return(output);
}