#ifndef _park_h
#define _park_h

#include "headfile.h"

#define START_T         10000
#define T_OUT_PARK1      1900
#define T_OUT_PARK2      2400

//��������
extern int8   flag_start;          //�������
extern int8   flag_open_reed;      //���θɻɹ�
extern int8   reed_state;          //�ɻɹ�״̬
extern int8   flag_end;            //��ʼͣ����־λ
extern uint16 T_outku;    //����ֱ�ߺʹ�Ƕ�ʱ
extern uint16 J_outku;
extern uint16 T_inku_wait;
extern uint16 T_inku_J;
extern uint16 T_inku_S;
extern uint16 T_inku;

extern uint16 S_daoku;
extern uint8 Library_selection;

//��������
extern void Handle_Barn_Out(uint8 type);
extern void Reed(void);
extern void In_park(uint8 type);

#endif