#include "stm32f0xx.h"
#include "stm32f0xx_it.h"


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}


void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

//根据方向控制信号改变计数器的上下计数方向
void EXTI0_1_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    if(DIRIN==1)
			LL_TIM_SetCounterMode(TIM1,LL_TIM_COUNTERMODE_UP);
		else
			LL_TIM_SetCounterMode(TIM1,LL_TIM_COUNTERMODE_DOWN);
  }
}
//电机使能信号外部中断函数
void EXTI2_3_IRQHandler(void)
{
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
		if(ENIN==1)
		{
			y=*(volatile uint16_t*)(ReadAngle()*2+0x08008000);
			s_sum=y;
			s=0;
			s_1=0;
      r=y;
      r_1=r;
      y_1=y;
			yw=y;  
			yw_1=yw;
			wrap_count=0;
			LL_TIM_SetCounter(TIM1,0);
			LL_TIM_EnableCounter(TIM1);
			enmode=1;
		}
		else
		{
			enmode=0;
			LL_TIM_DisableCounter(TIM1);
			LL_TIM_OC_SetCompareCH1(TIM3,0);  
			LL_TIM_OC_SetCompareCH2(TIM3,0);  
		}
  }
}


void TIM6_IRQHandler(void)
{
  if(LL_TIM_IsActiveFlag_UPDATE(TIM6) == 1)
  {
	LL_TIM_ClearFlag_UPDATE(TIM6);
	LL_IWDG_ReloadCounter(IWDG);//清看门狗
    if(enmode==1)
	{
	  if(closemode==1) 
	  {    
		y=*(volatile uint16_t*)(ReadAngle()*2+0x08008000);//读出编码器的角度位置值
        s=LL_TIM_GetCounter(TIM1);//读出计数器计数的外部step控制脉冲数
	    if(s-s_1<-32500)
		  s_sum+=stepangle*65000;
	    else if(s-s_1>32500)
	      s_sum-=stepangle*65000;
		r=s_sum+stepangle*s;//溢出处理后根据电子齿轮算出指令电机位置
	    s_1=s;
		
        if(y-y_1>8192) 
	      wrap_count--;      
        else if(y-y_1<-8192) 
	      wrap_count++; 
        yw=y+16384*wrap_count;//编码器的角度位置值溢出处理后根据圈数算出实际电机位置值            
	    e=r-yw;//误差值
        if(e>1638)//误差值大小限制
        {
		  e=1638;
		  LED_H;
		}
        else if(e<-1638)
        {
		  e=-1638;
		  LED_H;
		}
		else
          LED_L;
			
        iterm+=ki*e/32;//积分项计算
		if(iterm>UMAXSUM) //积分饱和限制
	      iterm=UMAXSUM;
        else if(iterm<-UMAXSUM) 
		  iterm=-UMAXSUM; 
    
            
        dterm=LPFA*dterm/128-LPFB*kd*(yw-yw_1)/8;//微分项计算
        u=(kp*e+iterm+dterm)/128;//PID三项计算值
	    
		advance=(yw-yw_1)*3;
        y_1=y;  
        yw_1=yw;
	
		if(u>0)            
        {		  
		  y+=(82+advance);//电流矢量等于1.8度加上前馈角
		}
        else if(u<0)
        {
		  y-=(82-advance);
		  u=-u;
		}
        if(u>UMAXCL)     
		  u=UMAXCL;//电流矢量最大值限制
        Output(y,u);    
      }          
      else 
	  {		
		s=LL_TIM_GetCounter(TIM1);
	    if(s-s_1<-32500)
		  s_sum+=stepangle*65000;
	    else if(s-s_1>32500)
	      s_sum-=stepangle*65000;
		r=s_sum+stepangle*s;
	    s_1=s;
		
		if(r==r_1)
		{
		  hccount++;
		  if(hccount>=10000)
		    hccount=10000;
		}
		else
		  hccount=0;
		
    if(hccount>=10000)//1s自动进入半流模式
		  Output(r,UMAXOP/2);
		else
		  Output(r,UMAXOP);
		r_1=r;
	  }	 	  
    } 
  }
}


