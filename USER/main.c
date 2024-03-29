
#include "rs485.h"
#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	 
#include "24cxx.h"
#include "includes.h" 	 
#include "adc.h"
#include "timer.h"
#include "485ANN.h"
#include "ht1621.h"
#include "key.h"
//32
#include "lcd.h"//测试用
/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				64
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);	
 			   
//LED任务
//设置任务优先级
#define TAKE_TASK_PRIO       			7 
//设置任务堆栈大小
#define TAKE_STK_SIZE  		    		64
//任务堆栈
OS_STK TAKE_TASK_STK[TAKE_STK_SIZE];
//任务函数
void Heartbeat_task(void *pdata);

//接收任务
//设置任务优先级
#define Receive_TASK_PRIO       			8 
//设置任务堆栈大小
#define Receive_STK_SIZE  		    		64
//任务堆栈
OS_STK Receive_TASK_STK[Receive_STK_SIZE];
//任务函数
void  Receive_task(void *pdata);





//主任务
//设置任务优先级
 #define MAIN_TASK_PRIO 4 
//设置任务堆栈大小
#define MAIN_STK_SIZE  					128
//任务堆栈	
OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
//任务函数
void main_task(void *pdata);

//信号量集任务
//设置任务优先级

//设置任务堆栈大小
#define MASTER_STK_SIZE  		 		64
//任务堆栈	
OS_STK MASTER_TASK_STK[MASTER_STK_SIZE];
//任务函数
 void master_task(void *pdata);
 

#define MYLED_TASK_PRIO       			13 
//设置任务堆栈大小
#define MYLED_STK_SIZE  		    		64
//任务堆栈
OS_STK MYLED_TASK_STK[MYLED_STK_SIZE];
//任务函数
void myled_task(void *pdata);

#define SETID_TASK_PRIO       			2 
//设置任务堆栈大小
#define SETID_STK_SIZE  		    		64
//任务堆栈
OS_STK SETID_TASK_STK[SETID_STK_SIZE];
//任务函数
void SETID_task(void *pdata);



  	   
extern box mybox;
  
extern  u16  dog_clock;

extern OS_EVENT * RS485_MBOX,*RS485_STUTAS_MBOX;			//	rs485邮箱信号量

extern OS_EVENT *Heartbeat;			 //心跳信号量
extern OS_EVENT *master_led_task;
extern u8 cont;//用于更改主机号的记次数器
extern  u8 token[33];//主机号令牌

extern status_box mystatus;

extern status_list_node system_status_list_1[33];

extern status_list_node system_status_list_2[33];

extern idle_list sort_idle_list_1[33];
extern idle_list sort_idle_list_2[33];
extern busy_list sort_busy_list_1[33];
extern busy_list sort_busy_list_2[33];


extern u8 hguestnum,gonglvshishu;
extern u32 idle_time,dianliuzhi;
extern u16 wugongkvar;
extern s8 L_C_flag;
extern u8 id_num;
//接收缓存区

//////////////////////////////////////////

u8 led_lock=0;
u8 init=1;
int subcontrol(u8,u8);

#define THREE_OFFSET  1

#define SIZE_1 5
#define SIZE_2 5
#define SIZE_3 5
#define WORK_STATUS_1	 0//0为没有工作  1为工作  2为坏掉，初始化为0
#define WORK_STATUS_2    0
#define WORK_STATUS_3 0
#define WORK_TIME_1 0
#define WORK_TIME_2	0
#define WORK_TIME_3 0
/////////////////////////////////////////////
int main(void)
 {	 
  
	 
	
	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
// 	LED_Init();			     //LED端口初始化
/*************************/
		delay_us(500000);
	HT1621_Init();
	KEY_Init();          //初始化与按键连接的硬件接口  
	AT24CXX_Init();			//IIC初始化
	Adc_Init();
/************************************/

///		uart_init(9600);LCD_Init();	                                                              //调试显示
	RS485_Init(9600);	//初始化RS485
	TIM4_Int_Init(9999,7199);//10Khz的计数频率，计数5K次为500ms 
	TIM3_Cap_Init(0XFFFF,72-1);	//以1Mhz的频率计数	  //开发板用LCD时，必须注掉此举，引脚冲突
	 initmybox();
	 init_mystatus(SIZE_1,SIZE_2,SIZE_3,WORK_STATUS_1,WORK_STATUS_2,WORK_STATUS_3,WORK_TIME_1,WORK_TIME_2,WORK_TIME_3);
	 
	OSInit();  	 			//初始化UCOSII
			  
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	    
}							    
//开始任务
void start_task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;  	    
	pdata = pdata; 	
	 Heartbeat=OSSemCreate(0);
	 RS485_MBOX=OSMboxCreate((void*)0);
	 RS485_STUTAS_MBOX=OSMboxCreate((void*)0);
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    			   
 	OSTaskCreate(master_task,(void *)0,(OS_STK*)&MASTER_TASK_STK[MASTER_STK_SIZE-1],MASTER_TASK_PRIO);	 				   
 	OSTaskCreate(Heartbeat_task,(void *)0,(OS_STK*)&TAKE_TASK_STK[TAKE_STK_SIZE-1],TAKE_TASK_PRIO);
	OSTaskCreate(Receive_task,(void *)0,(OS_STK*)&Receive_TASK_STK[Receive_STK_SIZE-1],Receive_TASK_PRIO);
	OSTaskCreate(myled_task,(void *)0,(OS_STK*)&MYLED_TASK_STK[MYLED_STK_SIZE-1],MYLED_TASK_PRIO);
	OSTaskCreate(SETID_task,(void *)0,(OS_STK*)&SETID_TASK_STK[SETID_STK_SIZE-1],SETID_TASK_PRIO);
 	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
//LED任务

 /**************从机任务**********************/
void Receive_task(void *pdate)//从机任务
{   u8 err;
	 u8 *msg;
	 int flag1,flag2;
	
    while(1)
    	{
		 if(mybox.master==1)OSTaskSuspend(Receive_TASK_PRIO);
	 msg=(u8 *)OSMboxPend(RS485_MBOX,0,&err);//接收到有数据
	 flag1=rs485_trans_order(msg);
	 dog_clock=10;	 
	 if(flag1==0);/***主机不是给本机通信，信息舍弃***/
	 if(flag1==1)/***是本机信息***/
	 	{		//LED1=!LED1;	  
		       dog_clock=10;	 
	 	      flag2=subcontrol(mybox.relay,mybox.message);
		       if(flag2==0);/******心跳信息，主机活着，不必启动从新定位主机函数，返回给主机本从机也存在    喂狗*********/
               if(flag2==1);/*****下位机，控制命令，喂狗*****/		  
	 	}

      if(led_lock==0){temperature();key_lcd();}
	}
	}
 /**************主机任务**********************/
  void master_task(void *pdata)	  //主机任务
  {	  OS_CPU_SR cpu_sr=0;
      u8 go=6;
	  u32 i;
	  // u8 *msg,err;
	  u8 try_cont=0;
	  for(i=1;i<33;i++){set_statuslist_1(i,0,0,0,0);set_statuslist_2(i,0,0,0,0);set_statuslist_3(i,0, 0, 0, 1);}//初始化两个状态队�
   while(1)
   	{
  	if(mybox.master==1)
     {	hguestnum=111;
	if(init==1)
		{
       delay_time(5000);//初始化延时，使系统稳定
	   init=0;
		}
	   if(go==0)
	  { myled(); 
	//    LED0=!LED0;
	   delay_time(1);
		 if(((gonglvshishu)<90)&&(L_C_flag==1))
		 	{
		 	offset_idlepower();
		       }
		 if(((gonglvshishu)>95)&&(L_C_flag==1))
		 	{
                        unload_power(system_status_list_1,system_status_list_2);
		       }
	        if((idle_time>500)&&(gonglvshishu>90&&gonglvshishu<95)&&(L_C_flag==1))
                {idle_time=0;turn_power(system_status_list_1,system_status_list_2);}
		if(L_C_flag==0){C_unload_power(system_status_list_1, system_status_list_2);}
	   }
		if(go==1)
		{
		{OSSemPost(Heartbeat);delay_ms(100);try_cont++;}  //把延时封装成函数
		   	if(try_cont==20)
		  for(i=1;i<33;i++)
		  {	//LED0=!LED0;  
	       order_trans_rs485(mybox.myid,i,1,1,1);
		   delay_us(20000);
		  	order_trans_rs485(mybox.myid,i,1,2,1); 
		   delay_us(20000);
		   }
		   if(try_cont==40)
		   for(i=1;i<33;i++)
		   {
			// LED0=!LED0; 
		   order_trans_rs485(mybox.myid,i,1,1,0);
		  delay_us(20000);
		   order_trans_rs485(mybox.myid,i,1,2,0);
		    delay_us(50000);
			try_cont=0;
			}
		}	
		if(go==2)
			   {
                 myled(); 
				                   
                        	led_on_off(0);		 //关闭从机led功能，防止通信控制信息的丢失
			   for(i=1;i<33;i++)		 //收集从机状态
               { inquiry_slave_status(i);	
	         }
			   //下面是用LCD屏测试回馈信息功能
				 for(i=1;i<33;i++)
				   {
				   LCD_ShowxNum(5+1*25,i*15,system_status_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,system_status_list_1[i].size,3,16,0X80);
				   LCD_ShowxNum(5+3*25,i*15,system_status_list_1[i].work_status,3,16,0X80);	
			       LCD_ShowxNum(5+4*25,i*15,system_status_list_1[i].work_time,3,16,0X80);
				   LCD_ShowxNum(5+5*25,i*15,system_status_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+6*25,i*15,system_status_list_2[i].size,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,system_status_list_2[i].work_status,3,16,0X80);	
			       LCD_ShowxNum(5+8*25,i*15,system_status_list_2[i].work_time,3,16,0X80);
				   }
			 	
				 for(i=1;i<33;i++)
					{
				   if(system_status_list_1[i].work_status==0){order_trans_rs485(mybox.myid,i,1,1,1);  delay_us(10000);}//验证从机9的控制和工作计时
				   if(system_status_list_2[i].work_status==0){order_trans_rs485(mybox.myid,i,1,2,1);  delay_us(10000);}//				     
				 	}
                       
				
				     //打开从机led功能
                        	led_on_off(1);
		       }
			 	if(go==3){// LED0=!LED0;
				  order_trans_rs485(mybox.myid,2,2,0,0);
				  delay_us(10000);
				  }
				
				if(go==4){ 
					s8 c=0,d=0;
                       myled() ;             
			led_on_off(0);		 //关闭从机led功能，防止通信控制信息的丢失

							 for(i=1;i<33;i++)
					{
			   if(system_status_list_1[i].work_status==0){order_trans_rs485(mybox.myid,i,1,1,1);  delay_us(10000);}//验证从机9的控制和工作计时
				   if(system_status_list_2[i].work_status==0){order_trans_rs485(mybox.myid,i,1,2,1);  delay_us(10000);}//				     
				 	}

			   for(i=1;i<33;i++)		 //收集从机状态
               { inquiry_slave_status(i);	
	         }
                             c=sort_busynode_list_asc(sort_busy_list_2,system_status_list_2);
                             d=sort_busynode_list_asc(sort_busy_list_1, system_status_list_1);
			   //下面是用LCD屏测试回馈信息功能
				 for(i=1;i<=d;i++)
				   {
				   LCD_ShowxNum(5+1*25,i*15,sort_busy_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,sort_busy_list_1[i].size,3,16,0X80);
				   LCD_ShowxNum(5+3*25,i*15,sort_busy_list_1[i].work_time,3,16,0X80);	
			 //         LCD_ShowxNum(5+4*25,i*15,idle_time,3,16,0X80);
//				    LCD_ShowxNum(5+5*25,i*15,dianliuzhi,3,16,0X80);
  
					}
					for(i=1;i<=c;i++)
					{
				   LCD_ShowxNum(5+6*25,i*15,sort_busy_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,sort_busy_list_2[i].size,3,16,0X80);
				   LCD_ShowxNum(5+8*25,i*15,sort_busy_list_2[i].work_time,3,16,0X80);	
			       
				   }

		//		 d=sort_idlenode_list(sort_idle_list_2,system_status_list_2);			 

		//		    for(i=1;i<=d;i++)
			//	   {
                         //         LCD_ShowxNum(5+1*25,i*15,sort_idle_list_2[i].myid,3,16,0X80);
				//   LCD_ShowxNum(5+2*25,i*15,sort_idle_list_2[i].size,3,16,0X80);

//				   }
				 	
/*							 for(i=1;i<33;i++)
					{
			   if(system_status_list_1[i].work_status==1){order_trans_rs485(mybox.myid,i,1,1,0);  delay_us(10000);}//验证从机9的控制和工作计时
				   if(system_status_list_2[i].work_status==1){order_trans_rs485(mybox.myid,i,1,2,0);  delay_us(10000);}//				     
				 	}

    */                   
				
				     //打开从机led功能
                        	led_on_off(1);
		 

					}

		if(go==5)
			{

                                                      s8 d=0,c=0;
								  led_on_off(0);
						   for(i=1;i<33;i++)		 //收集从机状态
               { inquiry_slave_status(i);	
	         }			  
					
		c=sort_idlenode_list(sort_idle_list_1, system_status_list_1);
		 d=sort_idlenode_list(sort_idle_list_2,system_status_list_2);			 

				    for(i=1;i<=d;i++)
				   {
                                  LCD_ShowxNum(5+1*25,i*15,sort_idle_list_2[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+2*25,i*15,sort_idle_list_2[i].size,3,16,0X80);

				   }
				for(i=1;i<=c;i++)
					{                        LCD_ShowxNum(5+6*25,i*15,sort_idle_list_1[i].myid,3,16,0X80);
				   LCD_ShowxNum(5+7*25,i*15,sort_idle_list_1[i].size,3,16,0X80);

					}
				 led_on_off(1);	
		}

     if(go==6)
     	{
          	while(1)
	{
               delay_time(1);
		   myled(); 
     // order_trans_rs485(mybox.myid,mybox.myid,1,1,1);
	//	 delay_us(1000000);
		// order_trans_rs485(mybox.myid,mybox.myid,1,1,0);
		// delay_us(1000000);
		// order_trans_rs485(mybox.myid,mybox.myid,1,2,1);
		// delay_us(1000000);
		// order_trans_rs485(mybox.myid,mybox.myid,1,2,0);
		// delay_us(1000000);
	        // order_trans_rs485(mybox.myid,mybox.myid,1,1,1);
       // order_trans_rs485(mybox.myid,mybox.myid,1,2,1);
	 


	  
	 
	}	 

	 }


	}
		                                      //启动接收程
	if(mybox.master==0)
	{
		OS_ENTER_CRITICAL();
		 OSTaskResume(Receive_TASK_PRIO );//启动从机任务状态
		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
		OS_EXIT_CRITICAL();	
	}
	}//while
 	}

void myled_task(void *pdata)
{
while(1)
{
if(mybox.master==1)OSTaskSuspend( MYLED_TASK_PRIO);
if(led_lock==0)
	{temperature();key_lcd();}
delay_ms(100);
}
}




/********************************************/
void SETID_task(void *pdata)
{
        OS_CPU_SR cpu_sr=0;  	    	
          while(1)
          	{
		  id_num=AT24CXX_ReadOneByte(0x0010);
		if(id_num<1||id_num>33)
			{            		
                                      mybox.master=2;
			             OS_ENTER_CRITICAL();
                      		OSTaskSuspend(MASTER_TASK_PRIO );//挂起主机任状态.
                      		OSTaskSuspend( MYLED_TASK_PRIO);
                                   OSTaskSuspend(Receive_TASK_PRIO);
						OS_EXIT_CRITICAL();
					HT595_Send_Byte(YELLOW_YELLOW|background_light_on);
                                      myled(); 									  
								   
		       }
               else if(id_num<=32&&id_num>=1)
               	{ 
                                  mybox.master=0;
				      mybox.myid=id_num;
				HT595_Send_Byte((GREEN_GREEN)|background_light_on);
				   OS_ENTER_CRITICAL();
		 OSTaskResume(MASTER_TASK_PRIO );//启动主机任务状态
		 OSTaskResume(MYLED_TASK_PRIO );//启动显示任务状态
		 OSTaskResume(Receive_TASK_PRIO );//启动从机任务状态
               OSTaskSuspend(SETID_TASK_PRIO);
			   OS_EXIT_CRITICAL();	
			 }

		  }

}

/******************************************/


 /***********************************/

int subcontrol(u8 i,u8 j)//给下下位机放指令	 
{ 
 if(mybox.send==0)//心态脉搏
   	{
	   //LED1=!LED1;
	return 0;
    }
   if(mybox.send==1) //下位机控制
   	{ 	 // LED1=!LED1;
  	if(i==1&&j==1)
        {  GPIO_ResetBits(GPIOA,GPIO_Pin_0);
set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.size[2],1,mystatus.work_status[1],mystatus.work_status[2],mystatus.work_time[0],mystatus.work_time[1],mystatus.work_time[2]);
	}
    if(i==1&&j==0)
{GPIO_SetBits(GPIOA,GPIO_Pin_0);
set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.size[2],0,mystatus.work_status[1],mystatus.work_status[2],0,mystatus.work_time[1],mystatus.work_time[2]);
}  
if(i==2&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.size[2],mystatus.work_status[0],1,mystatus.work_status[2],mystatus.work_time[0],mystatus.work_time[1],mystatus.work_time[2]);LED0=!LED0; }
    if(i==2&&j==0){GPIO_SetBits(GPIOA,GPIO_Pin_8);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.size[2],mystatus.work_status[0],0,mystatus.work_status[2],mystatus.work_time[0],0,mystatus.work_time[2]);}
   	if(i==3&&j==1){GPIO_ResetBits(GPIOA,GPIO_Pin_0);set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.size[2],mystatus.work_status[0],mystatus.work_status[1],1,mystatus.work_time[0],mystatus.work_time[1],mystatus.work_time[2]);LED0=!LED0; }
    if(i==3&&j==0)
  { GPIO_SetBits(GPIOA,GPIO_Pin_0);
set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],mystatus.size[2],mystatus.work_status[0],mystatus.work_status[1],0,mystatus.work_time[0],mystatus.work_time[1],0);
     }
	  LIGHT(mystatus.work_status[0],mystatus.work_status[1]);
	return 1;
   	}
if(mybox.send==2)//查看从机状态
 {
 //	LED1=!LED1;
  status_trans_rs485(&mystatus);
return 2;
 }
 if(mybox.send==3) //关闭刷新led屏幕
 {
 led_lock=1;
 return 3;
 }
 if(mybox.send==4) //打开刷新led屏幕
 {
  led_lock=0;
 return 4;
 }
return 5; //操作失败
}


 
