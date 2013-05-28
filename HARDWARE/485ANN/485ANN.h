#ifndef __485ANN_H
#define __485ANN_H			 
#include "sys.h"	 
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include <math.h>
#include "adc.h"
#include "timer.h"								  
#include "Ht1621.h"
	  		  	
 
 typedef struct  
{ 
  u8 start;
  u8 myid;      //��������ID��
  u8 source;
  u8 destination; //Ŀ�ĵ�����
  u8 send;      //�Ƿ��Ƿ�������1Ϊ�ǣ�0Ϊ����
  u8 relay;    //�ڼ��������
  u8 message;     //������Ϣ
  u8 master;      //��������
  u8 end;   
}box;
#define LEN_control 15
#define LEN_status 10
#define MASTER_TASK_PRIO       			3 


//#define RS485_TX_EN		PGout(9)	//485ģʽ����.0,����;1,����.��������
#define RS485_TX_EN		PBout(15)	//485ģʽ����.0,����;1,����.��������
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.

#define TIME_OUT 500

 typedef struct  
{ 
  u8 myid;      //��������ID��
  u8 size[3];      //������λǧ��
  u8 work_status[3];    //����״̬ 1 ΪͶ�빤����0 Ϊû�й���
  u8 work_time[3];     //����ʱ��  
  u8 three_offset;
}status_box;


#define status_LEN 4




 typedef struct  
{ 
  u8 myid;      //��������ID��
  u8 size;      //������λǧ��
  u8 work_status;    //����״̬ 1 ΪͶ�빤����0 Ϊû�й���
  u8 work_time;     //����ʱ��   
  u8 three_offset;
}status_list_node;



typedef struct
{
  u8 myid;
  u8 size;

}idle_list;

typedef struct
{
  u8 myid;
  u8 group;
  u8 size;
}turn_node;

typedef struct
{
  u8 myid;
  u8 size;
  u8 work_time;

}busy_list;

 //////////////////////////////////////////////////////////////////// 
/*
typedef struct
{
   u8 myid;
   u8 work_time;
   u8 size;

 }time_list;
*/
void turn_master_id(u8);
void initmybox(void);
void TIM4_Int_Init(u16,u16);
void TIM3_Cap_Init(u16 arr,u16 psc);
void order_trans_rs485(u8,u8,u8,u8,u8);
int rs485_trans_order(u8 *);
void rs485_trans_status(u8 *);
void status_trans_rs485(status_box *);
void set_now_mystatus(u8 ,u8 ,u8 ,u8,u8,u8,u8,u8,u8,u8);
void init_mystatus(u8 ,u8 ,u8,u8,u8,u8,u8,u8,u8);
void set_statuslist_1(u8,u8,u8,u8,u8);
void set_statuslist_2(u8,u8,u8,u8,u8);
void set_statuslist_3(u8 ,u8 ,u8,u8 ,u8 );
void delay_time(u32);//��ϵͳ����ʱ������time*450ms
void inquiry_slave_status(u8);//��ѯ�ӻ�״̬�����浽�ӻ�״̬���У�����id��Ҫ��ѯ�Ĵӻ���
u16 power_computer(void);
void offset_idlepower(void);
void turn_power(status_list_node *,status_list_node *);
void unload_power(status_list_node *,status_list_node *);
void C_unload_power(status_list_node *, status_list_node *);
void gonglvyinshu(void);
void temperature(void);   //�������¶ȼ��
s8 sort_busynode_list(busy_list *,status_list_node *);
s8 sort_threebusynode_list(busy_list *,status_list_node *);
s8 sort_busynode_list_asc(busy_list *,status_list_node *);
s8 sort_idlenode_list(idle_list *,status_list_node *);
s8 sort_threeidlenode_list(idle_list *,status_list_node *);
void myled(void);
void try(void);
u16 comp_16(u16 ,u16 );
void led_on_off(u8 ) ;
extern int subcontrol(u8,u8);
void heartbeat(u8);
void Alarm(void);
void key_lcd(void);
void LIGHT(u8,u8);
s8 turn_idlenode_list(turn_node *,status_list_node *,status_list_node *);//�����������(��������С�ɴ�С���У����ؿ��нڵ����)

#endif	   
















