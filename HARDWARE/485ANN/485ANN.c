#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "485ANN.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "key.h"
/***********************************************************************/
u16 RS485_RX_BUF[64]; 		//接收缓冲,最大64个字节
//接收到的数据长度
u8 RS485_RX_CNT=0;  
//模式控制
 u16  dog_clock=10;

 OS_EVENT * RS485_MBOX,* RS485_STUTAS_MBOX;			//	rs485邮箱信号量
 OS_EVENT *Heartbeat;			 //心跳信号量
OS_EVENT *master_led_task;
u8 cont=0;//用于更改主机号的记次数器
u32 life_time_1=0;
u32 life_time_2=0;
U32 life_time_3=0;
u8 turn_flag=1;//轮休使用变量
s8 turn_label_idle=0;//轮休使用变量

box mybox;
status_box mystatus;
idle_list sort_idle_list_1[33];
idle_list sort_idle_list_2[33];
turn_node turn_idle_list[65];
busy_list sort_busy_list_1[33];
busy_list sort_busy_list_2[33];

status_list_node system_status_list_1[33];
status_list_node system_status_list_2[33];
status_list_node system_status_list_3[33];

u8 idle_done_nodelist_1[33];
u8 idle_done_nodelist_2[33];
u8 done_list1_flag=0,done_list2_flag=0;
u8 done_count_1=0,done_count_2=0;


//u8 rs485buf[LEN_control];//发送控制信息
u8 rs485buf[LEN_control];//发送控制信息
u8 statusbuf[LEN_status];//发送状态信息

u32 idle_time=0;

u8 alarm_lock=0;
/****************************************************************/
u8 si[]={0,2,3,5,7,9,10,12,14,16,        //0~9
	   	17,19,21,22,24,26,28,29,31,33,  //10~19
		34,36,37,39,41,42,44,45,47,48,  //20~29
		50,52,53,54,56,57,59,60,62,63,	//30~39
		64,66,67,68,69,71,72,73,74,75,	//40~49
		77,78,79,80,81,82,83,84,85,86,	//50~59
		87,87,88,89,90,91,91,92,93,93,	//60~69
		94,95,95,96,96,97,97,97,98,98,	//70~79
		98,99,99,99,99,100,100,100,100,100,100	//80~90
		};
u8 co[]={100,100,100,100,100,99,99,99,99,98, //0~9
          98,98,97,97,97,96,96,95,95,94,	  //10~19
          93,93,92,91,91,90,89,88,87,87,	  //20~29
          86,85,84,83,82,81,80,79,78,77,	  //30~39
          75,74,73,72,71,69,68,67,66,64,	  //40~49
          63,62,60,59,57,56,54,53,52,50,	  //50~59
          48,47,45,44,42,41,39,37,36,34,	  //60~69
          33,31,29,28,26,24,22,21,19,17,	  //70~79
          16,14,12,10,9,7,5,3,2,0			  //80~90
		  };


/**********************测试无功功率 数据*************************************/


/************************************************************/
u16 wugong_95,wugong_computer;

extern u8 id_num;
extern u8 grafnum,tempshuzhi,gonglvshishu;
extern u16 dianya_zhi,	wugongkvar,k;
extern u32	dianliuzhi;
s8 L_C_flag;//感性容性标准变量
/*****************************************************/
 void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}
 
 void TIM4_IRQHandler(void)   //TIM4中断
{	 
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
		{	  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志
	if(mybox.master==0)	
		{
		
		if(dog_clock==0)
		   { 
		   // LED0=!LED0;
			turn_master_id(mybox.myid);
			  cont++;
			}
			if(dog_clock>0){dog_clock--;cont=1;}
		 }
		 if (mystatus.work_status[0]==1)  //工作时间的计时
		    {  life_time_1++;
		       if(life_time_1==10)
			   { mystatus.work_time[0]++;
			     life_time_1=0;
				  }
			        if(mystatus.work_time[0]==37)mystatus.work_time[0]=39;
				  if(mystatus.work_time[0]==254)mystatus.work_time[0]=0;
			   }
		 	
		 if(mystatus.work_status[1]==1)
		 	{ life_time_2++;
              if(life_time_2==10)
			  	{  mystatus.work_time[1]++;
			       life_time_2=0;
              	         }   
                   if(mystatus.work_time[1]==37)mystatus.work_time[1]=39;
			 if(mystatus.work_time[1]==254)mystatus.work_time[1]=0;	   
			    
		     }

		 		 if (mystatus.work_status[2]==1)  //工作时间的计时
		    {  life_time_3++;
		       if(life_time_3==10)
			   { mystatus.work_time[2]++;
			     life_time_3=0;
				  }
			        if(mystatus.work_time[2]==37)mystatus.work_time[2]=39;
				  if(mystatus.work_time[2]==254)mystatus.work_time[2]=0;
			   }

		   if (mystatus.work_status[0]==0)  //工作时间清零
		    {   mystatus.work_time[0]=0;}
		    if (mystatus.work_status[1]==0)  //工作时间清零
		    {   mystatus.work_time[1]=0;}

		    if (mystatus.work_status[2]==0)  //工作时间清零
		    {   mystatus.work_time[2]=0;}

			if(mybox.master==1)
		       {  idle_time++;
			if(idle_time==65535)idle_time=0;
			}
		}
   	OSIntExit();  
}
//////////////////////////////////////////////////////////////////////////////////
void TIM3_Cap_Init(u16 arr,u16 psc)
{	NVIC_InitTypeDef NVIC_InitStructure;	 
	RCC->APB1ENR|=1<<1;   	//TIM3 时钟使能 
	RCC->APB2ENR|=1<<3;    	//使能PORTB时钟  
	 
	GPIOB->CRL&=0XFFFFFF00;	//PB0 PB1清除之前设置  
	GPIOB->CRL|=0X00000088;	//PB0 PB1输入   
	GPIOB->ODR&=~(1<<0);		//PB0 PB1下拉
	GPIOB->ODR&=~(1<<1);
	  
 	TIM3->ARR=arr;  		//设定计数器自动重装值   
	TIM3->PSC=psc;  		//预分频器 

	TIM3->CCMR2|=1<<0;		//CC3S=01 	选择输入端 IC3映射到TI3上
 	TIM3->CCMR2|=0<<4; 		//IC3F=0000 配置输入滤波器 不滤波
 	TIM3->CCMR2|=0<<2;  	//IC3PS=00 	配置输入分频,不分频 

	TIM3->CCER|=0<<9; 		//CC3P=0	上升沿捕获1
	TIM3->CCER|=1<<8; 		//CC3E=1 	允许捕获计数器1的值到捕获寄存器中

	TIM3->DIER|=1<<3;   	//允许捕获3中断				
	TIM3->DIER|=1<<0;   	//允许更新1中断	

	TIM3->CCMR2|=1<<8;		//CC4S=01 	选择输入端 IC4映射到TI4上
 	TIM3->CCMR2|=0<<12; 	//IC4F=0000 配置输入滤波器 不滤波
 	TIM3->CCMR2|=0<<10; 	//IC4PS=00 	配置输入分频,不分频 

	TIM3->CCER|=0<<13; 		//CC4P=0	上升沿捕获
	TIM3->CCER|=1<<12; 		//CC4E=1 	允许捕获计数器的值到捕获寄存器中

	TIM3->DIER|=1<<4;   	//允许捕获4中断				
	TIM3->DIER|=1<<0;   	//允许更新中断
	TIM3->CR1|=0x01;    	//使能定时器2
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //从优先级4级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	   
}

//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到高电平;1,已经捕获到高电平了.
//[5:0]:捕获高电平后溢出的次数
u8  TIM3CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM3CH1_CAPTURE_VAL;	//输入捕获值
u16	TIM3CH1_CAPTURE_PHA;	//输入捕获值
//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 		    
	u16 tsr;
	tsr=TIM3->SR;
 	if((TIM3CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(tsr&0X01)//溢出
		{	    
			if(TIM3CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM3CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM3CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM3CH1_CAPTURE_VAL=0XFFFF;
				}else TIM3CH1_CAPTURE_STA++;
			}	 
		}
		if(tsr&0x10)//捕获1发生捕获事件
		{	
			if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM3CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			    TIM3CH1_CAPTURE_VAL=TIM3->CCR4;	//获取当前的捕获值.
	 			TIM3->CCER&=~(1<<1);			//CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM3CH1_CAPTURE_STA=0;			//清空
				TIM3CH1_CAPTURE_VAL=0;
				TIM3CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
	 			TIM3->CNT=0;					//计数器清空
			  	TIM3->CCER&=~(1<<1);			//CC1P=0 设置为上升沿捕获
			}		    
		}
		if(tsr&0x08)
		{
		 	if(TIM3CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
			    TIM3CH1_CAPTURE_PHA=TIM3->CCR3;	//获取当前的捕获值.
			}
		}			     	    					   
 	}
	TIM3->SR=0;//清除中断标志位 	    
}


//////////////////////////////////////////////////////////
		void USART2_IRQHandler(void)
{
 
	u8 RS485_RX_BUF[64];
	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
      #endif
 		
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	 
	 			 
		 RS485_RX_BUF[RS485_RX_CNT++]=USART_ReceiveData(USART2); 	//读取接收到的数据
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='&'){RS485_RX_BUF[0]='&'; RS485_RX_CNT=1;}
		if(RS485_RX_BUF[RS485_RX_CNT-1]=='*')
		{
				RS485_RX_CNT=0;

				
				if(RS485_RX_BUF[1]=='#'){OSMboxPost(RS485_STUTAS_MBOX,(void*)&RS485_RX_BUF);}
				  else OSMboxPost(RS485_MBOX,(void*)&RS485_RX_BUF);
		} 
	}  	
	#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif

} 

void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	RS485_TX_EN=1;			//设置为发送模式
  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	RS485_TX_EN=0;				//设置为接收模式	

}

void initmybox()//初始化自身信息
{  	 
  
  mybox.master=0;
 mybox.start='&';
 mybox.myid=AT24CXX_ReadOneByte(0x0010);
///mybox.myid=1;
 mybox.source=0;
 mybox.destination=0;
 mybox.send=0;
 mybox.relay=0;
 mybox.message=0;
 mybox.end='*';	
						
}

void turn_master_id(u8 id)//改变当前整个系统中主机的ID号
{
   u8 i,flag=0;
	{ 
	  flag=cont;
      if(id==(flag)){
	  	for(i=1;i<33;i++)
			{ order_trans_rs485(mybox.myid,i,0,0,0);
		     delay_us(10000);
			}//及时告知其他slave机器，已有主机
         mybox.master=1;
	    OSTaskResume(MASTER_TASK_PRIO);
		cont=1;
	  }
	 //  LED1=!LED1;
	  
      }
   }






 int rs485_trans_order(u8 *tx_r485)//解析由主机发送过来的信号，并发送给下位机
{ 
  dianya_zhi=comp_16(tx_r485[6],tx_r485[7]);
  dianliuzhi=comp_16(tx_r485[8],tx_r485[9]);
  wugongkvar=comp_16(tx_r485[10],tx_r485[11]);
  //tempshuzhi=tx_r485[12];
  gonglvshishu=tx_r485[12];
   if(mybox.myid==tx_r485[2]||tx_r485[2]==0)//判断是否是发给本机的信息或是广播信息
   	{
   	 mybox.source=tx_r485[1];
   	 mybox.send=tx_r485[3];
     mybox.relay=tx_r485[4];
     mybox.message=tx_r485[5];
     return 1;
   	}
   else return 0;
}

 void order_trans_rs485(u8 source,u8 destination, u8 send,u8 relay,u8 message)//主机程序，主机命令解析成RS485信息，发送给目的从机
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
      rs485buf[0]='&';//协议头
	rs485buf[1]=source;
	rs485buf[2]=destination;
	rs485buf[3]=send;
	rs485buf[4]=relay;
	rs485buf[5]=message;
	rs485buf[6]=(dianya_zhi & (uint16_t)0x00FF);
	rs485buf[7]=((dianya_zhi & (uint16_t)0xFF00)>>8);
	rs485buf[8]=(dianliuzhi& (uint16_t)0x00FF);
	rs485buf[9]=((dianliuzhi& (uint16_t)0xFF00)>>8);
	rs485buf[10]=(wugongkvar& (uint16_t)0x00FF);
	rs485buf[11]=((wugongkvar& (uint16_t)0xFF00)>>8);
	//rs485buf[12]=tempshuzhi;
	rs485buf[12]=gonglvshishu;
	rs485buf[13]='*';//协议尾
	RS485_Send_Data(rs485buf,14);//发送5个字节
	if(destination==source){mybox.send=send;subcontrol(relay, message);}//如果信息发给的自己
	OS_EXIT_CRITICAL();	
}

u16 comp_16(u16 a,u16 b)
{
u16 value=0;
value=((a&0x00FF)+((b<<8)&0xFF00));
return value;
}


void Heartbeat_task(void *pdata)//master任务发送任务
{		// u8 key;
        u8 i=0;
		u8 err;
	//  u8 rs485buf[5];
	while(1)
	{
	OSSemPend(Heartbeat,0,&err);
		for(i=1;i<10;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}		 
				   
	}
}

void heartbeat(u8 t)
{	u8 i;
for(i=0;i<=t;i++)
		{	
	       order_trans_rs485(mybox.myid,0,0,0,0);
		    delay_us(10000);
		  // LCD_ShowxNum(60+i*32,190,0,7,16,0X80);
		}	
}










void led_on_off(u8 on_off) //参数值1 为打开led ，0为关闭led
{
u8 i;
if(on_off==1)
    {
        for(i=1;i<65;i++)
	   { order_trans_rs485(mybox.myid,0,4,0,0);
             delay_us(10000);
          }	 
    }
if(on_off==0)
    {
        for(i=1;i<65;i++)
	   { order_trans_rs485(mybox.myid,0,3,0,0);
             delay_us(10000);
          }	 
    }
}


/*****************************回馈信息函数********************************************/


void init_mystatus(u8 size_1,u8 size_2,u8 size_3,u8 work_status_1,u8 work_status_2,u8 work_status_3,u8 work_time_1,u8 work_time_2,u8 work_time_3)
{
mystatus.myid= mybox.myid;
mystatus.size[0]=size_1;
mystatus.size[1]=size_2;
mystatus.work_status[0]=work_status_1;
mystatus.work_status[1]=work_status_2;
mystatus.work_time[0]=work_time_1;
mystatus.work_time[1]=work_time_2;
if(THREE_OFFSET==1)
   {
   mystatus.size[2]=size_3;
  mystatus.work_status[2]=work_status_3;
  mystatus.work_time[2]=work_time_3;
  mystatus.three_offset=1;
    }
else 
     {
     mystatus.size[2]=0;
     mystatus.work_status[2]=2;
     mystatus.work_time[2]=0;
	 mystatus.three_offset=0;
     }
}



void set_now_mystatus(u8 myid,u8 size_1,u8 size_2,u8 size_3,u8 work_status_1,u8 work_status_2,u8 work_status_3,u8 work_time_1,u8 work_time_2,u8  work_time_3)
{
mystatus.myid=myid;
mystatus.size[0]=size_1;
mystatus.size[1]=size_2;
mystatus.size[2]=size_3;
mystatus.work_status[0]=work_status_1;
mystatus.work_status[1]=work_status_2;
mystatus.work_stauts[2]=work_status_3;
mystatus.work_time[0]=work_time_1;
mystatus.work_time[1]=work_time_2;
mystatus.work_time[2]=work_time_3;
}



 void status_trans_rs485(status_box *mystatus)//从机程序
{  	 OS_CPU_SR cpu_sr=0;
    OS_ENTER_CRITICAL();
	if(THREE_OFFSET==0)
		{
       statusbuf[0]='&';
	statusbuf[1]='#';
	statusbuf[2]=mystatus->myid;
	statusbuf[3]=mystatus->size[0];
	statusbuf[4]=mystatus->size[1];
	statusbuf[5]=mystatus->work_status[0];
	statusbuf[6]=mystatus->work_status[1];
	statusbuf[7]=mystatus->work_time[0];
	statusbuf[8]=mystatus->work_time[1];
	statusbuf[9]=mystatus->three_offset;
	statusbuf[10]='*';
	RS485_Send_Data(statusbuf,11);//发送10个字节
		}
	if(THREE_OFFSET==1)
		{
                  statusbuf[0]='&';
	  statusbuf[1]='#';
	  statusbuf[2]=mystatus->myid;
	  statusbuf[3]=mystatus->size[0];
	  statusbuf[4]=mystatus->size[1];
	  statusbuf[5]=mystatus->work_status[0];
	  statusbuf[6]=mystatus->work_status[1];
	  statusbuf[7]=mystatus->work_time[0];
	  statusbuf[8]=mystatus->work_time[1];
	  statusbuf[9]=mystatus->size[2];
	  statusbuf[10]=mystatus->work_status[2];
	 statusbuf[11]=mystatus->work_time[2];
	 statusbuf[12]=mystatus->three_offset;
	  statusbuf[13]='*';
     	  RS485_Send_Data(statusbuf,14);//发送13个字节

	      }
	OS_EXIT_CRITICAL();	
}


 void rs485_trans_status(u8 *tx_r485)//主机程序，主机命令解析成RS485信息，发送给目的从机
 	{
                 if(tx_r485[12]==0)
                 	{
	   system_status_list_1[tx_r485[2]].myid=tx_r485[2];
   	   system_status_list_1[tx_r485[2]].size=tx_r485[3];
   	   system_status_list_1[tx_r485[2]].work_status=tx_r485[5];
          system_status_list_1[tx_r485[2]].work_time=tx_r485[7];
	    system_status_list_1[tx_r485[2]].three_offset=tx_r485[12];

	   system_status_list_2[tx_r485[2]].myid=tx_r485[2];
   	   system_status_list_2[tx_r485[2]].size=tx_r485[4];
   	   system_status_list_2[tx_r485[2]].work_status=tx_r485[6];
          system_status_list_2[tx_r485[2]].work_time=tx_r485[8];
	   system_status_list_2[tx_r485[2]].three_offset=tx_r485[12];
	   
                 	}
		if(tx_r485[12]==1)
			{
            system_status_list_1[tx_r485[2]].myid=tx_r485[2];
   	   system_status_list_1[tx_r485[2]].size=tx_r485[3];
   	   system_status_list_1[tx_r485[2]].work_status=tx_r485[5];
          system_status_list_1[tx_r485[2]].work_time=tx_r485[7];
	    system_status_list_1[tx_r485[2]].three_offset=tx_r485[12];

	   system_status_list_2[tx_r485[2]].myid=tx_r485[2];
   	   system_status_list_2[tx_r485[2]].size=tx_r485[4];
   	   system_status_list_2[tx_r485[2]].work_status=tx_r485[6];
          system_status_list_2[tx_r485[2]].work_time=tx_r485[8];
	   system_status_list_2[tx_r485[2]].three_offset=tx_r485[12];

           system_status_list_3[tx_r485[2]].myid=tx_r485[2];
		system_status_list_3[tx_r485[2]].size=tx_r485[9];
		system_status_list_3[tx_r485[2]].work_status=tx_r485[10];
		system_status_list_3[tx_r485[2]].work_time=tx_r485[11];
		system_status_list_3[tx_r485[2]].three_offset=tx_r485[12];
           
		        }
	   
		 // LED0=!LED0;
   }


void set_statuslist_1(u8 id,u8 size,u8 work_status,u8 work_time,u8 three_offset)
{
       system_status_list_1[id].myid=id;
   	   system_status_list_1[id].size=size;
   	   system_status_list_1[id].work_status=work_status;
       system_status_list_1[id].work_time=work_time;
	   system_status_list_1[id].three_offset=three_offset;

}

void set_statuslist_2(u8 id,u8 size,u8 work_status,u8 work_time,u8 three_offset)
{
       system_status_list_2[id].myid=id;
   	   system_status_list_2[id].size=size;
   	   system_status_list_2[id].work_status=work_status;
       system_status_list_2[id].work_time=work_time;
           system_status_list_2[id].three_offset=three_offset;
}


void set_statuslist_3(u8 id,u8 size,u8 work_status,u8 work_time,u8 three_offset)
{
       system_status_list_3[id].myid=id;
   	   system_status_list_3[id].size=size;
   	   system_status_list_3[id].work_status=work_status;
       system_status_list_3[id].work_time=work_time;
        system_status_list_3[id].three_offset=three_offset;
}


void offset_idlepower()  //功率补偿函数，三个参数 无功功率 功率因数 空闲队列
{
 u8 i,j;
 s8 label_idle1,label_idle2;
  turn_flag=1;
  led_on_off(0);
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_idle1=sort_idlenode_list(sort_idle_list_1,system_status_list_1);
  label_idle2=sort_idlenode_list(sort_idle_list_2,system_status_list_2);

	for(j=1;j<=label_idle1;j++)
		{
                         if(done_list1_flag==0)
                            {

				   for(i=0;i<33;i++)
				   	    {
                                          idle_done_nodelist_1[i]=0;

				           }
				   done_list1_flag=1;
			       }
			delay_time(5);
				myled();
		              if(gonglvshishu>90)break;
		                 else{
                    if((wugong_computer)>=(sort_idle_list_1[j].size))
			             {    if(idle_done_nodelist_1[sort_idle_list_1[j].myid]==0)
			                   {
			                   order_trans_rs485(mybox.myid,sort_idle_list_1[j].myid,1,1,1);delay_us(10000);
                                          idle_done_nodelist_1[sort_idle_list_1[j].myid]=1;
					    }
					}
					done_count_1++;
					if(done_count_1==label_idle1)
						{done_count_1=0;done_list1_flag=0;}
		   	                  }
		   }

	
 	if(gonglvshishu<90)
 		{
	for(j=1;j<=label_idle2;j++)
		{
		                         if(done_list2_flag==0)
                            {

				   for(i=0;i<33;i++)
				   	    {
                                          idle_done_nodelist_2[i]=0;

				           }
				   done_list2_flag=1;
			       }

                                     delay_time(5);
                                     myled();
		   if(gonglvshishu>90)break;
		   else{
                          if((wugong_computer)>=(sort_idle_list_2[j].size))
                          	            {  if(idle_done_nodelist_2[sort_idle_list_2[j].myid]==0)
                          	                {     
						  order_trans_rs485(mybox.myid,sort_idle_list_2[j].myid,1,2,1);delay_us(10000);
                                                 idle_done_nodelist_2[sort_idle_list_2[j].myid]=1;
                          	                }
						  done_count_2++;
						  	if(done_count_2==label_idle2)
						  		{done_count_2=0;done_list2_flag=0;}
						  
						  }
		   	 }
	    }

 		}
		led_on_off(1);

}

void turn_power(status_list_node *list_1,status_list_node *list_2)//到功率因素满足稳定条件后，进入电容器轮休函数
{
u8 i,j,k,t,q;
s8 label_busy1,label_busy2;
 if(turn_flag==1)
   {
   			led_on_off(0);
   for(i=1;i<33;i++)inquiry_slave_status(i); 
   			led_on_off(1);
   turn_label_idle=turn_idlenode_list(turn_idle_list,list_1,list_2);//得到空闲队列
   }  
   turn_flag=0;

if(turn_label_idle!=0)
{	      led_on_off(0);
   for(i=1;i<33;i++)inquiry_slave_status(i);   			
    label_busy1=sort_busynode_list(sort_busy_list_1,list_1);//刷新list_1的busy表的每个工作节点的工作时间
    label_busy2=sort_busynode_list(sort_busy_list_2,list_2);//刷新list_2的busy表的每个工作节点的工作时间

      for(i=1;i<=label_busy1;i++)
	  	{ if(sort_busy_list_1[i].work_time>=TIME_OUT)
      	      {  
				for(j=1;j<=turn_label_idle;j++)
                	{  
                       if(sort_busy_list_1[i].size==turn_idle_list[j].size)
                       	{ order_trans_rs485(mybox.myid,turn_idle_list[j].myid,1,turn_idle_list[j].group,1);delay_us(10000);
					                                           delay_us(100000);//实验看灯延时，真实情况注掉
                             order_trans_rs485(mybox.myid,sort_busy_list_1[i].myid,1,1,0);delay_us(10000);
						 	{ k=sort_busy_list_1[i].myid;
                                                    t=sort_busy_list_1[i].size;
                                                       sort_busy_list_1[i].work_time=0;
							  for(q=j;q<turn_label_idle;q++)
							  	{turn_idle_list[q].myid=turn_idle_list[q+1].myid;
                                                          turn_idle_list[q].size=turn_idle_list[q+1].size;
								turn_idle_list[q].group=turn_idle_list[q+1].group;
							      }
							    turn_idle_list[turn_label_idle].myid=k;
							    turn_idle_list[turn_label_idle].size=t;
							   turn_idle_list[turn_label_idle].group=1;
						     } 
                                           break;//防止本循环中对i，重复匹配投切

					   }
				    }
      	}
      	}

	      for(i=1;i<=label_busy2;i++)	
   { if(sort_busy_list_2[i].work_time>=TIME_OUT)
      	      {  
				for(j=1;j<=turn_label_idle;j++)
                	{  
                       if(sort_busy_list_2[i].size==turn_idle_list[j].size)
                       	{ order_trans_rs485(mybox.myid,turn_idle_list[j].myid,1,turn_idle_list[j].group,1);delay_us(10000);
					                                           delay_us(100000);//实验看灯延时，真实情况注掉
                             order_trans_rs485(mybox.myid,sort_busy_list_2[i].myid,1,2,0);delay_us(10000);
						 	{ k=sort_busy_list_2[i].myid;
                                                    t=sort_busy_list_2[i].size;
                                                       sort_busy_list_2[i].work_time=0;
							  for(q=j;q<turn_label_idle;q++)
							  	{turn_idle_list[q].myid=turn_idle_list[q+1].myid;
                                                          turn_idle_list[q].size=turn_idle_list[q+1].size;
								turn_idle_list[q].group=turn_idle_list[q+1].group;
							      }
							    turn_idle_list[turn_label_idle].myid=k;
							    turn_idle_list[turn_label_idle].size=t;
							   turn_idle_list[turn_label_idle].group=2;
						     } 
                                           break;//防止本循环中对i，重复匹配投切

					   }
				    }
      	}
      	}
   // if(label_idle1==0&&label_idle2==0)break;//无空闲队列 
 led_on_off(1);
   }
}

void unload_power(status_list_node *list_1,status_list_node *list_2)
{
 u8 i,j;
 s8 label_busy1,label_busy2;
  turn_flag=1;
  led_on_off(0);
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_busy2=sort_busynode_list_asc(sort_busy_list_2,list_2);
 label_busy1=sort_busynode_list_asc(sort_busy_list_1,list_1);


	for(j=1;j<=label_busy1&&label_busy1>0;j++)
		{
                     delay_time(5);
			myled();//计算功率，电压电流与显示按键分开
		   if(gonglvshishu<95)break;
		   else {	
						if((wugong_computer-wugong_95)>sort_busy_list_1[label_busy1].size)
				   {order_trans_rs485(mybox.myid,sort_busy_list_1[label_busy1].myid,1,1,0);delay_us(10000);label_busy1--;}

						if((wugong_computer-wugong_95)<=sort_busy_list_1[j].size)
				   {order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,1,1,0);delay_us(10000);}

		          }
	    }

if(gonglvshishu>95)
 {
	for(j=1;j<=label_busy2&&label_busy2>0;j++)
		{
                        delay_time(5);
	                 myled();//计算功率，电压电流与显示按键分开
		   if(gonglvshishu<95)break;
		   else{	                               
                                						if((wugong_computer-wugong_95)>sort_busy_list_2[label_busy2].size)
				   {order_trans_rs485(mybox.myid,sort_busy_list_2[label_busy2].myid,1,2,0);delay_us(10000);label_busy2--;}

			   if((wugong_computer-wugong_95)<=sort_busy_list_2[j].size)
		   	       {order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0);delay_us(10000);}
                            

		           }
        }

}
				led_on_off(1);

}

void C_unload_power(status_list_node *list_1,status_list_node *list_2)
{
 u8 i,j;
 s8 label_busy1,label_busy2;
  turn_flag=1;
  led_on_off(0);
for(i=1;i<33;i++)inquiry_slave_status(i);
 label_busy2=sort_busynode_list_asc(sort_busy_list_2,list_2);
 label_busy1=sort_busynode_list_asc(sort_busy_list_1,list_1);

	for(j=1;j<=label_busy1&&label_busy1!=0;j++)
		{

                     delay_time(5);
			myled();//计算功率，电压电流与显示按键分开
		   if(L_C_flag==1)break;
		   else {	
                            						if((wugong_computer)>sort_busy_list_1[label_busy1].size)
				   {order_trans_rs485(mybox.myid,sort_busy_list_1[label_busy1].myid,1,1,0);delay_us(10000);label_busy1--;}

                                                            if((wugong_computer)<=sort_busy_list_1[j].size)
			{order_trans_rs485(mybox.myid,sort_busy_list_1[j].myid,1,1,0);delay_us(10000);}
		          }
	    }

if(L_C_flag==0)
 {
	for(j=1;j<=label_busy2&&label_busy2!=0;j++)
		{
                            delay_time(5);
				myled();//计算功率，电压电流与显示按键分开
		   if(L_C_flag==1)break;
		   else{	
                                                            if((wugong_computer)>sort_busy_list_2[label_busy2].size)
				   {order_trans_rs485(mybox.myid,sort_busy_list_2[label_busy2].myid,1,2,0);delay_us(10000);label_busy2--;}

														  if((wugong_computer)<=sort_busy_list_2[j].size)
			{order_trans_rs485(mybox.myid,sort_busy_list_2[j].myid,1,2,0);delay_us(10000);}

		   }
        }


}
				led_on_off(1);

}

s8 sort_idlenode_list(idle_list *sort_idle_list,status_list_node *list)//空闲有序队列(按容量大小由大到小排列，返回空闲节点个数)
{
   u8 i,j=1,k,t,flag=0;
   s8 count=0;
   for(i=1;i<33;i++){sort_idle_list[i].myid=0;sort_idle_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==0&&list[i].size!=0&&list[i].three_offset==0)
                      { sort_idle_list[j].myid=list[i].myid;
				        sort_idle_list[j].size=list[i].size;
					    j++;
						count++;
						if(flag==0)flag=1;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_idle_list[i].size;
	   k=sort_idle_list[i].myid;
	   for(j=i-1;j>=1&&t>sort_idle_list[j].size;j--)
	   	{sort_idle_list[j+1].myid=sort_idle_list[j].myid;
         sort_idle_list[j+1].size=sort_idle_list[j].size;
	    }
	   sort_idle_list[j+1].myid=k;
	   sort_idle_list[j+1].size=t;

      }
for(i=1;i<count;i++)
   {if(sort_idle_list[i].myid==mybox.myid)
           {   t=sort_idle_list[i].size;
	        k=sort_idle_list[i].myid;
             for(j=i;j<count;j++)
              	{sort_idle_list[j].size=sort_idle_list[j+1].size;
                       sort_idle_list[j].myid=sort_idle_list[j+1].myid;
			 }
			 sort_idle_list[count].size=t;
			 sort_idle_list[count].myid=k;
			 break;
           }
    }
   	}
    return count;
}

/**********************************************************************************/
s8 sort_threeidlenode_list(idle_list *sort_idle_list,status_list_node *list)//空闲有序队列(按容量大小由大到小排列，返回空闲节点个数)
{
   u8 i,j=1,k,t,flag=0;
   s8 count=0;
   for(i=1;i<33;i++){sort_idle_list[i].myid=0;sort_idle_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==0&&list[i].size!=0&&list[i].three_offset==1)
                      { sort_idle_list[j].myid=list[i].myid;
				        sort_idle_list[j].size=list[i].size;
					    j++;
						count++;
						if(flag==0)flag=1;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_idle_list[i].size;
	   k=sort_idle_list[i].myid;
	   for(j=i-1;j>=1&&t>sort_idle_list[j].size;j--)
	   	{sort_idle_list[j+1].myid=sort_idle_list[j].myid;
         sort_idle_list[j+1].size=sort_idle_list[j].size;
	    }
	   sort_idle_list[j+1].myid=k;
	   sort_idle_list[j+1].size=t;

      }
for(i=1;i<count;i++)
   {if(sort_idle_list[i].myid==mybox.myid)
           {   t=sort_idle_list[i].size;
	        k=sort_idle_list[i].myid;
             for(j=i;j<count;j++)
              	{sort_idle_list[j].size=sort_idle_list[j+1].size;
                       sort_idle_list[j].myid=sort_idle_list[j+1].myid;
			 }
			 sort_idle_list[count].size=t;
			 sort_idle_list[count].myid=k;
			 break;
           }
    }
   	}
    return count;
}






/*********************************************************************************/

/**************将第一组空闲队列和第二组空闲队列组成一组进行排序轮 休***************************/


s8 turn_idlenode_list(turn_node *turn_idle_list,status_list_node *list_1,status_list_node *list_2)//空闲有序队列(按容量大小由大到小排列，返回空闲节点个数)
{
   u8 i,j=1,k,t,g,flag=0;
   s8 count=0;
   for(i=1;i<65;i++){turn_idle_list[i].myid=0;turn_idle_list[i].size=0;turn_idle_list[i].group=0;}
   for(i=1;i<33;i++){
   	              if(list_1[i].work_status==0&&list_1[i].size!=0)
                      { turn_idle_list[j].myid=list_1[i].myid;
				        turn_idle_list[j].size=list_1[i].size;
						turn_idle_list[j].group=1;
					          j++;
						count++;
						if(flag==0)flag=1;//如果没有空闲节点
   	              	  }
                    }

for(i=1;i<33;i++)
		{
   	              if(list_2[i].work_status==0&&list_2[i].size!=0)
                      { turn_idle_list[j].myid=list_2[i].myid;
				        turn_idle_list[j].size=list_2[i].size;
						turn_idle_list[j].group=2;
					          j++;
						count++;
						if(flag==0)flag=1;//如果没有空闲节点
   	              	  }
                    }
   
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=turn_idle_list[i].size;
	   k=turn_idle_list[i].myid;
	   g=turn_idle_list[i].group;
	   for(j=i-1;j>=1&&t>turn_idle_list[j].size;j--)
	   	{turn_idle_list[j+1].myid=turn_idle_list[j].myid;
                turn_idle_list[j+1].size=turn_idle_list[j].size;
		  turn_idle_list[j+1].group=turn_idle_list[j].group;
	    }
	            turn_idle_list[j+1].myid=k;
	           turn_idle_list[j+1].size=t;
			turn_idle_list[j+1].group=g;

      }
for(i=1;i<count;i++)
   {if(turn_idle_list[i].myid==mybox.myid)
           {   t=turn_idle_list[i].size;
	        k=turn_idle_list[i].myid;
		g=turn_idle_list[i].group;
             for(j=i;j<count;j++)
              	{turn_idle_list[j].size=turn_idle_list[j+1].size;
                       turn_idle_list[j].myid=turn_idle_list[j+1].myid;
			turn_idle_list[j].group=turn_idle_list[j+1].group;
			 }
			 turn_idle_list[count].size=t;
			 turn_idle_list[count].myid=k;
			 turn_idle_list[count].group=g;
			 break;
           }
    }
   	}
    return count;
}








/***************************************************************************************************************************/







/*
void sort_timenode_list(time_list *sort_time_list,status_list_node *list)//时间有序队列(按工作时间由大到小排列)
{
u8 i,j=1,k,t,s;
for(i=1;i<33;i++){sort_time_list[i].myid=0;sort_time_list[i].work_time=0;sort_time_list[i].size=0;}
 for(i=1;i<33;i++){
   	              if(list[i].myid!=0)
                     {sort_time_list[j].myid=list[i].myid;
				      sort_time_list[j].work_time=list[i].work_time;
					  sort_time_list[j].size=list[i].size;
					  j++;
   	              	 }
                    }
   for(i=2;i<33;i++)
   	{
       t=sort_time_list[i].work_time;
	   k=sort_time_list[i].myid;
	   s=sort_time_list[i].size;
	   for(j=i-1;j>=1&&t>sort_time_list[j].work_time;j--)
	   	{sort_time_list[j+1].myid=sort_time_list[j].myid;
         sort_time_list[j+1].work_time=sort_time_list[j].work_time;
		 sort_time_list[j+1].size=sort_time_list[j].size;
	    }
	   sort_time_list[j+1].myid=k;
	   sort_time_list[j+1].work_time=t;
       sort_time_list[j+1].size=s;
   }
}
*/
s8 sort_busynode_list(busy_list *sort_busy_list,status_list_node *list)//忙碌有序队列(按工作时间大小由大到小排列)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   s8 count=0,count_time=0;
   for(i=1;i<33;i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==1&&list[i].size!=0&&list[i].three_offset==0)
                      { sort_busy_list[j].myid=list[i].myid;
				        sort_busy_list[j].size=list[i].size;
						sort_busy_list[j].work_time=list[i].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t>sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
  return count;
}

/****************************************************************************/
s8 sort_threebusynode_list(busy_list *sort_busy_list,status_list_node *list)//忙碌有序队列(按工作时间大小由大到小排列)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   s8 count=0,count_time=0;
   for(i=1;i<33;i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==1&&list[i].size!=0&&list[i].three_offset==1)
                      { sort_busy_list[j].myid=list[i].myid;
				        sort_busy_list[j].size=list[i].size;
						sort_busy_list[j].work_time=list[i].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t>sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
  return count;
}


/**************************************************************************/


s8 sort_busynode_list_asc(busy_list *sort_busy_list,status_list_node *list)//忙碌有序队列(按工作容量大小由大到小排列)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   s8 count=0,count_time=0;
   for(i=1;i<33;i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==1&&list[i].size!=0&&list[i].three_offset==0)
                      { sort_busy_list[j].myid=list[i].myid;
				        sort_busy_list[j].size=list[i].size;
						sort_busy_list[j].work_time=list[i].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t<sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
  return count;
}

/*******************************************************/
s8 sort_threebusynode_list_asc(busy_list *sort_busy_list,status_list_node *list)//忙碌有序队列(按工作容量大小由大到小排列)
{
   u8 i,j=1,g,f,k,t,w,flag=0;
   s8 count=0,count_time=0;
   for(i=1;i<33;i++){sort_busy_list[i].myid=0;sort_busy_list[i].size=0;}
   for(i=1;i<33;i++){
   	              if(list[i].work_status==1&&list[i].size!=0&&list[i].three_offset==1)
                      { sort_busy_list[j].myid=list[i].myid;
				        sort_busy_list[j].size=list[i].size;
						sort_busy_list[j].work_time=list[i].work_time;
					    j++;
						if(flag==0)flag=1;
						count++;
   	              	  }
                    }
   if(flag==1)
   	{
   for(i=2;i<=count;i++)
   	 {
       t=sort_busy_list[i].size;
	   k=sort_busy_list[i].myid;
	   w=sort_busy_list[i].work_time;
	   for(j=i-1;j>=1&&t<sort_busy_list[j].size;j--)
	   	{sort_busy_list[j+1].myid=sort_busy_list[j].myid;
         sort_busy_list[j+1].size=sort_busy_list[j].size;
		 sort_busy_list[j+1].work_time=sort_busy_list[j].work_time;
	    }
	   sort_busy_list[j+1].myid=k;
	   sort_busy_list[j+1].size=t;
       sort_busy_list[j+1].work_time=w;
     }
   /**********************************************/
   for(i=1;i<=count;i=i+count_time)
      { count_time=0;
        for(j=i;j<=count;j++)
        	{ if(sort_busy_list[i].size==sort_busy_list[j].size){count_time++;}
                     else break;
		}
		
		for(g=i+1;g<=i+count_time-1;g++)
			{
                               t=sort_busy_list[g].size;
	                        k=sort_busy_list[g].myid;
	                        w=sort_busy_list[g].work_time;
                 	   for(f=g-1;f>=i&&w>sort_busy_list[f].work_time;f--)
	   	{
	   	 sort_busy_list[f+1].myid=sort_busy_list[f].myid;
               sort_busy_list[f+1].size=sort_busy_list[f].size;
		 sort_busy_list[f+1].work_time=sort_busy_list[f].work_time;
	       }
	      sort_busy_list[f+1].myid=k;
	      sort_busy_list[f+1].size=t;
             sort_busy_list[f+1].work_time=w;
		       }

      }
           
   
   	}
  return count;
}




/******************************************************/





void delay_time(u32 time)
{ heartbeat(time);
}  //本系统的延时函数，time*10ms

void inquiry_slave_status(u8 id)   
  {  u8 *msg;
        u8 err;
   order_trans_rs485(mybox.myid,id,2,0,0);
  // delay_us(10000);
   msg=(u8 *)OSMboxPend(RS485_STUTAS_MBOX,OS_TICKS_PER_SEC/50,&err);
   if(err==OS_ERR_TIMEOUT)
   	{      
               	      set_statuslist_1(id,0,2,0,0);set_statuslist_2(id,0,2,0,0);set_statuslist_3(id, 0,2,0,0);
              
         }//(u8 id, u8 size, u8 work_status, u8 work_time) 
	else 
	  rs485_trans_status(msg);
	if(id==mybox.myid)
		{
                    if(mystatus.three_offset==0)
                    	{
	   set_statuslist_1(mystatus.myid,mystatus.size[0],mystatus.work_status[0],mystatus.work_time[0],0);//主机状态信息写入状态表
          set_statuslist_2(mystatus.myid,mystatus.size[1],mystatus.work_status[1],mystatus.work_time[1],0);
		  set_statuslist_3(id, 0,2,0,0);
                    	}
		  if(mystatus.three_offset==1)
		  	{
            set_statuslist_1(mystatus.myid,mystatus.size[0],mystatus.work_status[0],mystatus.work_time[0],1);//主机状态信息写入状态表
          set_statuslist_2(mystatus.myid,mystatus.size[1],mystatus.work_status[1],mystatus.work_time[1],1);
	             set_statuslist_3(mystatus.myid,mystatus.size[2],mystatus.work_status[2],mystatus.work_time[2],1);	  
           
		        }
		}
	} //查询从机状态并保存到从机状态表中，参数id是要查询的从机号






/*******************功率因素相关函数*****************************/

u16 power_computer()
{
        u16 i;
		u32 tempa=0,tempb=0;
		u16 adc_vx=0,adc_vmax=0,adc_ix=0,adc_imax=0;
		u8 phase_zhi=0;
		 float temp=0;

		id_num=AT24CXX_ReadOneByte(0x0010);
		key_idset();

		 for(i=0;i<120;i++)
	  	 {
	  	 adc_vx=Get_Adc_Average(ADC_Channel_1,10);
		  // adc_vx=Get_Adc(ADC_Channel_1);
		   if(adc_vx>adc_vmax)
		   adc_vmax=adc_vx;
	  	 }
	   for(i=0;i<120;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_CH4,10);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }
	  temp=(float)adc_vmax*(3.3/4096);
	 dianya_zhi=(u16)(518*temp-660);
	  temp=(float)adc_imax*(3.3/4096);
	  dianliuzhi=(u32)(60*temp-80);
	  adc_vmax=0;
	  adc_imax=0;
	  if(TIM3CH1_CAPTURE_STA&0X80)//完成一次采集
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//溢出时间总和
			tempa+=TIM3CH1_CAPTURE_VAL;		//得到TI1端信号周期时间


			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//溢出时间总和
			tempb+=TIM3CH1_CAPTURE_PHA;		//得到TI2 TI1上升沿时间差值即相位差时间

	 		 if(tempb<=5000)			   //感性负载正接
			 {
			 	phase_zhi=tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			  if((10000<=tempb)&&(tempb<=15000))			 //感性负载反接
			 {
			 	phase_zhi=((tempb*360)/tempa)-180;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((5000<tempb)&&(tempb<10000))	   //容性负载正接
			 {
				/*显示容性功率符号*/
			 	phase_zhi=180-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
			 if((15000<tempb)&&(tempb<20000))	   //容性负载反接
			 {
				/*显示容性功率符号*/
			 	phase_zhi=360-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
			 }
//			 wugongkvar=(uint16_t)(1.732*dianliuzhi*dianya_zhi*k*co[phase_zhi]);
			
			TIM3CH1_CAPTURE_STA=0;			//开启下一次捕获
			return gonglvshishu;
		}
	  return gonglvshishu;

//无功功率
}

void gonglvyinshu()
{
        u16 i;
		u32 tempa,tempb;
		u16 adc_vx,adc_vmax=0,adc_ix,adc_imax=0;
		u8 phase_zhi;
		 float temp;

		id_num=AT24CXX_ReadOneByte(0x0010);
	//	key_idset();

		 for(i=0;i<120;i++)
	  	 {
	  	 adc_vx=Get_Adc_Average(ADC_Channel_1,10);
		  // adc_vx=Get_Adc(ADC_Channel_1);
		   if(adc_vx>adc_vmax)
		   adc_vmax=adc_vx;
	  	 }
	   for(i=0;i<120;i++)
	  {
		 adc_ix=Get_Adc_Average(ADC_CH4,10);
		 if(adc_ix>adc_imax)
		 adc_imax=adc_ix;
	  }
	  temp=(float)adc_vmax*(3.3/4096);
	 dianya_zhi=(u16)(518*temp-679);
	  temp=(float)adc_imax*(3.3/4096);
	  dianliuzhi=(u32)(60*temp-80);
	  adc_vmax=0;
	  adc_imax=0;
	  	  	  if(dianliuzhi<7){dianliuzhi=0;gonglvshishu=100;}//滤除杂波，小于7时，说明已经无负载
            else{
	  if(TIM3CH1_CAPTURE_STA&0X80)//完成一次采集
		{
			tempa=TIM3CH1_CAPTURE_STA&0X3F;
			tempa*=65536;					//溢出时间总和
			tempa+=TIM3CH1_CAPTURE_VAL;		//得到TI1端信号周期时间


			tempb=TIM3CH1_CAPTURE_STA&0X3F;
			tempb*=65536;					//溢出时间总和
			tempb+=TIM3CH1_CAPTURE_PHA;		//得到TI2 TI1上升沿时间差值即相位差时间

	 		 if(tempb<=5000)			   //感性负载正接
			 {
			 	phase_zhi=tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
				L_C_flag=1;
			 }
			  if((10000<=tempb)&&(tempb<=15000))			 //感性负载反接
			 {
			 	phase_zhi=((tempb*360)/tempa)-180;
		   	 	gonglvshishu=si[phase_zhi];
				L_C_flag=1;
			 }
			 if((5000<tempb)&&(tempb<10000))	   //容性负载正接
			 {
				/*显示容性功率符号*/
			 	phase_zhi=180-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
				L_C_flag=0;
			 }
			 if((15000<tempb)&&(tempb<20000))	   //容性负载反接
			 {
				/*显示容性功率符号*/
			 	phase_zhi=360-tempb*360/tempa;
		   	 	gonglvshishu=si[phase_zhi];
				L_C_flag=0;
			 }
	  	}
			 wugongkvar=(uint16_t)((1.732*dianliuzhi*dianya_zhi*k*co[phase_zhi])/1000000);
			wugong_95= (uint16_t)((17.32*dianliuzhi*dianya_zhi*k*31)/1000000);//功率因素在0.95时的，无功功�
			wugong_computer=(uint16_t)((17.32*dianliuzhi*dianya_zhi*k*co[phase_zhi])/1000000);
                    wugongkvar=wugong_computer;
			TIM3CH1_CAPTURE_STA=0;			//开启下一次捕获
			
		
		}

//无功功率
}

void temperature()   //电容器温度检测
{
 u16 adc_tmp1=0,adc_tmp2=0;
       adc_tmp1=Get_Adc_Average(ADC_Channel_5,10);
	  adc_tmp2=Get_Adc_Average(ADC_Channel_6,10);
	  tempshuzhi=(u8)(258-((adc_tmp1*255)/4096));

}

void LIGHT(u8 status_1,u8 status_2)
{
if(status_1==0&&status_2==1)HT595_Send_Byte((GREEN_RED)|background_light_on);
if(status_1==1&&status_2==0)HT595_Send_Byte((RED_GREEN)|background_light_on);
if(status_1==0&&status_2==0)HT595_Send_Byte((GREEN_GREEN)|background_light_on);
if(status_1==1&&status_2==1)HT595_Send_Byte((RED_RED)|background_light_on);
if(status_1==2&&status_2==2)HT595_Send_Byte((YELLOW_YELLOW)|background_light_on);

}

void myled()
  {
gonglvyinshu();//计算功率，电压电流与显示按键分开
temperature();
key_lcd();
delay_ms(50);//没有延时，屏会死机
}

void Alarm(void)
{
	   if((tempshuzhi>=70||dianya_zhi>=440||dianya_zhi<=340)&&alarm_lock==0)
	   	{   
                                           if(mybox.master==1)//主机发布通知延时信息
								  {
								       delay_time(80);
								  }
					GPIO_SetBits(GPIOA,GPIO_Pin_0);
					delay_us(100000);
                    GPIO_SetBits(GPIOA,GPIO_Pin_8);
		       set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],2,2,0,0);
		       LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//过温常态
		       alarm_lock=1;

	   }
	 else if(tempshuzhi<70&&dianya_zhi<440&&dianya_zhi>340&&alarm_lock==1)
	 {
                             if(mybox.master==1)//主机
				 {    
				      delay_time(80);
				}
		    set_now_mystatus(mystatus.myid,mystatus.size[0],mystatus.size[1],0,0,0,0);
		    LIGHT(mystatus.work_status[0],mystatus.work_status[1]);//恢复常态
		    alarm_lock=0;
	 }

}
	 
void key_lcd()
{
key_idset();//按键与显示功能
Alarm();//是否需要报警
}

