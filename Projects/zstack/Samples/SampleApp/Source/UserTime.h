#ifndef CPT_TIMER_H 
#define CPT_TIMER_H 

#include <ioCC2530.h>
#define  TIMER50mS   35
#define  TIMER350S   350
#define  TIMER1S   1000
#define  TIMER2S   2000
#define  TIMER3S   3000

#define  KeyStop   0x05
#define  KeyRise   0x0C
#define  KeyFall   0x01
#define  KeyEnter  0x04
#define  KeyConfig 0x02
//#define DEBUG_BatteryBlind   //Debug模式 不进入休眠
//#define BF301
#define BF101
void Init_T1(void);
void Init_T3(void);
void Init_T4(void);
//00000000000000000000000000000000000000000000000000000000000
/*****************************************
//T3配置定义
*****************************************/
// Where _timer_ must be either 3 or 4
// Macro for initialising timer 3 or 4
//将T3/4配置寄存复位
#define TIMER34_INIT(timer)   \
   do {                       \
      T##timer##CTL   = 0x06; \
      T##timer##CCTL0 = 0x00; \
      T##timer##CC0   = 0x00; \
      T##timer##CCTL1 = 0x00; \
      T##timer##CC1   = 0x00; \
   } while (0)

//Macro for enabling overflow interrupt
//打开T3/4溢出中断
#define TIMER34_ENABLE_OVERFLOW_INT(timer,val) \
   (T##timer##CTL =  (val) ? T##timer##CTL | 0x08 : T##timer##CTL & ~0x08)
//启动T1
#define TIMER1_START(val)                         \
    (T1CCTL0 = (val) ? T1CCTL0 | 0X10 : T1CCTL0&~0X10)
//启动T3
#define TIMER3_START(val)                         \
    (T3CTL = (val) ? T3CTL | 0X10 : T3CTL&~0X10)

//时钟分步选择
#define TIMER3_SET_CLOCK_DIVIDE(val)              \
  do{                                             \
    T3CTL &= ~0XE0;                               \
      (val==2) ? (T3CTL|=0X20):                   \
      (val==4) ? (T3CTL|=0x40):                   \
      (val==8) ? (T3CTL|=0X60):                   \
      (val==16)? (T3CTL|=0x80):                   \
      (val==32)? (T3CTL|=0xa0):                   \
      (val==64) ? (T3CTL|=0xc0):                  \
      (val==128) ? (T3CTL|=0XE0):                 \
      (T3CTL|=0X00);             /* 1 */          \
  }while(0)

//Macro for setting the mode of timer3
//设置T3的工作方式
#define TIMER3_SET_MODE(val)                      \
  do{                                             \
    T3CTL &= ~0X03;                               \
    (val==1)?(T3CTL|=0X01):  /*DOWN            */ \
    (val==2)?(T3CTL|=0X02):  /*Modulo          */ \
    (val==3)?(T3CTL|=0X03):  /*UP / DOWN       */ \
    (T3CTL|=0X00);           /*free runing */     \
  }while(0)


/*****************************************
//T4配置定义
*****************************************/
// Where _timer_ must be either 3 or 4
// Macro for initialising timer 3 or 4
#define TIMER34_INIT(timer)   \
   do {                       \
      T##timer##CTL   = 0x06; \
      T##timer##CCTL0 = 0x00; \
      T##timer##CC0   = 0x00; \
      T##timer##CCTL1 = 0x00; \
      T##timer##CC1   = 0x00; \
   } while (0)

//Macro for enabling overflow interrupt
#define TIMER34_ENABLE_OVERFLOW_INT(timer,val) \
   (T##timer##CTL =  (val) ? T##timer##CTL | 0x08 : T##timer##CTL & ~0x08)



// Macro for configuring channel 1 of timer 3 or 4 for PWM mode.
#define TIMER34_PWM_CONFIG(timer)                 \
   do{                                            \
      T##timer##CCTL1 = 0x24;                     \
      if(timer == 3){                             \
         if(PERCFG & 0x20) {                      \
            IO_FUNC_PORT_PIN(1,7,IO_FUNC_PERIPH); \
         }                                        \
         else {                                   \
            IO_FUNC_PORT_PIN(1,4,IO_FUNC_PERIPH); \
         }                                        \
      }                                           \
      else {                                      \
         if(PERCFG & 0x10) {                      \
             IO_FUNC_PORT_PIN(2,3,IO_FUNC_PERIPH);\
         }                                        \
         else {                                   \
            IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH); \
         }                                        \
      }                                           \
   } while(0)

// Macro for setting pulse length of the timer in PWM mode
#define TIMER34_SET_PWM_PULSE_LENGTH(timer, value) \
   do {                                            \
      T##timer##CC1 = (BYTE)value;                 \
   } while (0)


// Macro for setting timer 3 or 4 as a capture timer
#define TIMER34_CAPTURE_TIMER(timer,edge)          \
   do{                                             \
      T##timer##CCTL1 = edge;                      \
      if(timer == 3){                              \
         if(PERCFG & 0x20) {                       \
            IO_FUNC_PORT_PIN(1,7,IO_FUNC_PERIPH);  \
         }                                         \
         else {                                    \
             IO_FUNC_PORT_PIN(1,4,IO_FUNC_PERIPH); \
         }                                         \
      }                                            \
      else {                                       \
         if(PERCFG & 0x10) {                       \
            IO_FUNC_PORT_PIN(2,3,IO_FUNC_PERIPH);  \
         }                                         \
        else {                                     \
           IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH);   \
        }                                          \
     }                                             \
  }while(0)

//Macro for setting the clock tick for timer3 or 4
#define TIMER34_START(timer,val)                         \
    (T##timer##CTL = (val) ? T##timer##CTL | 0X10 : T##timer##CTL&~0X10)

#define TIMER34_SET_CLOCK_DIVIDE(timer,val)        \
  do{                                             \
    T##timer##CTL &= ~0XE0;                               \
      (val==2) ? (T##timer##CTL|=0X20):                   \
      (val==4) ? (T##timer##CTL|=0x40):                   \
      (val==8) ? (T##timer##CTL|=0X60):                   \
      (val==16)? (T##timer##CTL|=0x80):                   \
      (val==32)? (T##timer##CTL|=0xa0):                   \
      (val==64) ? (T##timer##CTL|=0xc0):                  \
      (val==128) ? (T##timer##CTL|=0XE0):                 \
      (T##timer##CTL|=0X00);             /* 1 */          \
  }while(0)

//Macro for setting the mode of timer3 or 4
#define TIMER34_SET_MODE(timer,val)                \
  do{                                             \
    T##timer##CTL &= ~0X03;                               \
    (val==1)?(T##timer##CTL|=0X01):  /*DOWN            */ \
    (val==2)?(T##timer##CTL|=0X02):  /*Modulo          */ \
    (val==3)?(T##timer##CTL|=0X03):  /*UP / DOWN       */ \
    (T##timer##CTL|=0X00);           /*free runing */     \
  }while(0)

#endif 