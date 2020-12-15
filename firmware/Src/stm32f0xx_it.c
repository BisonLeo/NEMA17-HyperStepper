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

//���ݷ�������źŸı�����������¼�������
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
//���ʹ���ź��ⲿ�жϺ���
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
	LL_IWDG_ReloadCounter(IWDG);//�忴�Ź�
    if(enmode==1)
	{
	  if(closemode==1) 
	  {    
		y=*(volatile uint16_t*)(ReadAngle()*2+0x08008000);//�����������ĽǶ�λ��ֵ
        s=LL_TIM_GetCounter(TIM1);//�����������������ⲿstep����������
	    if(s-s_1<-32500)
		  s_sum+=stepangle*65000;
	    else if(s-s_1>32500)
	      s_sum-=stepangle*65000;
		r=s_sum+stepangle*s;//����������ݵ��ӳ������ָ����λ��
	    s_1=s;
		
        if(y-y_1>8192) 
	      wrap_count--;      
        else if(y-y_1<-8192) 
	      wrap_count++; 
        yw=y+16384*wrap_count;//�������ĽǶ�λ��ֵ�����������Ȧ�����ʵ�ʵ��λ��ֵ            
	    e=r-yw;//���ֵ
        if(e>1638)//���ֵ��С����
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
			
        iterm+=ki*e/32;//���������
		if(iterm>UMAXSUM) //���ֱ�������
	      iterm=UMAXSUM;
        else if(iterm<-UMAXSUM) 
		  iterm=-UMAXSUM; 
    
            
        dterm=LPFA*dterm/128-LPFB*kd*(yw-yw_1)/8;//΢�������
        u=(kp*e+iterm+dterm)/128;//PID�������ֵ
	    
		advance=(yw-yw_1)*3;
        y_1=y;  
        yw_1=yw;
	
		if(u>0)            
        {		  
		  y+=(82+advance);//����ʸ������1.8�ȼ���ǰ����
		}
        else if(u<0)
        {
		  y-=(82-advance);
		  u=-u;
		}
        if(u>UMAXCL)     
		  u=UMAXCL;//����ʸ�����ֵ����
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
		
        if(hccount>=10000)//1s�Զ��������ģʽ
		  Output(r,UMAXOP/2);
		else
		  Output(r,UMAXOP);
		r_1=r;
	  }	 	  
    } 
  }
}


