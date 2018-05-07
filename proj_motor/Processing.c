#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Timer.h"
#include "OS.h"
#include "IR.h"
#include "string.h"
#include "Ping.h"
#include "FIFO.h"
#include "can0.h"
#include "Processing.h"
#include "ST7735.h"
#include "pwm.h"
#include "uart.h"
 
uint32_t Steering;     // 625 to 3125
uint32_t SteeringMode; // 0 for increase, 1 for decrease
uint32_t Power;
uint32_t recv_speed;
uint8_t recv_dir_L;
uint8_t recv_dir_R;
uint8_t pre_recv_dir_L;
uint8_t pre_recv_dir_R;
int16_t offset;
uint16_t pixel_num;
static unsigned long offset_num; 
void uart_reciver(void) {
  
  unsigned long ir0, ir1, ir2, ir3, ping; 
  int16_t devia_speed;
  uint16_t divider = 1000;
  uint16_t  Left_PWM;
  uint16_t  Right_PWM;

  while(1) 
  {
     //uart2_GetMail(real_offset);
    OS_MailBox_Recv(&offset_num);
    pixel_num =  offset_num & 0xFFFF;
    offset = (offset_num >> 16) & 0xFFFF;
    //      rcv[0] direction 0~0xFF   7F:MID
//      rcv[1] f/b rcv[1]&0x80 speed 0~0x7F  
    
    Steering =  (uint32_t)(SERVOMID  - 5 * offset);   //0~255 --> MIN to MAX, resolution is 4.313
    if(Steering > SERVOMAX)
    {
        Steering = SERVOMAX; // go to center and
    }
    else if(Steering < SERVOMIN)
    {
        Steering =  SERVOMIN;
    }

    

    if((pixel_num < 15000) && (pixel_num > 30))
    {
      Left_Duty(8000 - 8*offset,1);
      Right_DutyB(8000 + 8*offset,1);
      Servo_Duty(Steering);
    }    else
    {
      Left_Duty(12450,1);
      Right_DutyB(12450,1);
      Servo_Duty(SERVOMID);
    }

  }
//    recv_dir_L =  (rcv[1] & 0x80) >> 7;        //take the MSB for direction 1->forward, 0-> backward
//    recv_dir_R =  (rcv[2] & 0x80) >> 7; 
//    recv_dir = 1;
//    recv_speed = 50;

//    devia_speed = rcv[0] - 0x7F;

//    if(pre_recv_dir_L != recv_dir_L)
//    {
//       if(recv_dir_L == 0)
//       {
//          Left_Init(12500, 12500,1);          // initialize PWM0, 100 Hz
//       }
//       else
//       {
//          Left_InitB(12500, 12500,1);          // initialize PWM0, 100 Hz
//       }
//    }
//    if(pre_recv_dir_R != recv_dir_R)
//    {
//       if(recv_dir_R == 0)
//       {
//          
//          Right_InitB(12500, 12500,1);   // initialize PWM0, 100 Hz
//       }
//       else
//       {
//          
//          Right_Init(12500, 12500,1);   // initialize PWM0, 100 Hz
//       }
//    }
////  if(recv_dir_L == 0)   //forward            
////  {
////      Left_PWM = 12500 - 98 * (rcv[1] & 0x7F);
////  }
////  if(recv_dir_R == 0)   //forward            
////  {
////      Right_PWM = 12500 - 98 *(rcv[2] & 0x7F);
////  }
////    else         //backward         cpomment for temp use
////    {
////      Left_PWM = 12500 - 98 * (rcv[1] & 0x7F);
////      Right_PWM = 12500 - 98 * (rcv[2] & 0x7F);
////    }
////    
//    
//    if(Left_PWM > POWERMAX)
//    {
//        Left_PWM = POWERMAX; // go to center and
//    }
//    else if(Left_PWM < POWERMIN)
//    {
//        Left_PWM =  POWERMIN;
//    }
//      if(Right_PWM > POWERMAX)
//    {
//        Right_PWM = POWERMAX; // go to center and
//    }
//    else if(Right_PWM < POWERMIN)
//    {
//        Right_PWM =  POWERMIN;
//    }
//    
//    if(recv_dir_L == 0)   //forward            
//    {
//      Left_Duty(Left_PWM,1);
//      
//    }
//    else if(recv_dir_L == 1)
//    {
//      Left_DutyB(Left_PWM,1);
//     
//    }//    if(recv_dir_R == 0)
//    {
//        Right_DutyB(Right_PWM,1);
//    }
//    else if(recv_dir_R == 1)
//    {
//         Right_Duty(Right_PWM,1);
//    }
//    pre_recv_dir_L = recv_dir_L;
//    pre_recv_dir_R = recv_dir_R;
//  }
}


void Processing_Init() {
//	OS_InitSemaphore(&CANLock, 0);
//	IR_Init();
//	Ping2_Init();
  OS_MailBox_Init();
  UART2_Init();
  CAN0_Open();
  OS_AddThread(&uart_reciver, 128, 1);
//	OS_AddThread(&DAS, 128, 1);
	// add processing thread
}
