#ifndef __ADC_H__
#define __ADC_H__

#include "headfile.h"

// 环岛检测电感阈值
#define IN_ANNULUS_H_LIMIT 80
// 出环检测电感阈值
#define OUT_ANNULUS_S_LIMIT 30
// 环岛积分距离
#define DISTANCE_ANNULUS_S 10
// 环岛打角积分
#define DISTANCE_ANNULUS_Z 0
// 环岛定时积分
#define DISTANCE_ANNULUS_T 500
// 出界判断
#define OUTSIDE 1

// 赛道类型判断
struct ROAD_TYPE
{
     int8 straight;         // 直道
     int8 bend;             // 弯道
     int8 annulus;          // 环岛
     int8 in_annulus_left;  // 入左环岛
     int8 in_annulus_right; // 入右环道
     int8 on_annulus_left;  // 在左环岛
     int8 on_annulus_right; // 在右环岛
     int8 out_annulus;      // 出环岛
     int8 in_park;          // 入库
};
extern struct ROAD_TYPE road_type;

// 变量声明
extern int16 adc_value[4];
extern int16 AD_V[4];
extern int16 adc_max[4];
extern int16 adc_min[4];
extern uint8 Left_Adc, Right_Adc, Left_Shu_Adc, Right_Shu_Adc;
extern float adc_valueM;
extern int8 NM;
extern uint16 annulus_s;
extern uint16 annulus_s2; // 环岛积分距离2
extern uint16 annulus_t;
extern uint16 annulus_z;
extern float Current_Dir;
extern int8 testflag;
extern int16 ADC_PWM;
extern int16 Set_gyr;
extern uint8 temp;


// 函数声明
void ADC_int(void);
void ADC_Collect(void);
void Data_current_analyze(void);
float Cha_bi_he(int16 data1, int16 data2, int16 x);
float Cha_bi_he_cha(int16 data1, int16 data2, int16 data3, int16 data4, int16 x, int16 y);
float Cha_x_bi_he(int16 data1, int16 data2, int16 data3, int16 data4);
void Road_type_judge(void);
void Annulus_handle(void);
float Direction_error(void);
void Out_protect(void);
void Get_deviation(void);
void Annulus_assist(void);

#endif