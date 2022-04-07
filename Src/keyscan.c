#define KEYSCAN_GLOBAL

#include "main.h"
#include "keyscan.h"


#define KEY_QTY      6      // 버튼의 수 
#define ON_TIME       5      // 버튼 눌림 인식시간 (10 x 10ms = 50ms) 
#define ON_MAX       5

#define REPEAT_KEY_START       250    //  40*12ms +40  반복키가 처음 발생하는 시점입니다.
#define CONTINUE_KEY_TIME      20     //  반복키가 발생하는 주기입니다.


volatile unsigned  char  KeyStatus[KEY_QTY]=  {0};       // 버튼 1개당 1 byte의 버퍼를 할당 
volatile  unsigned short KeyOnTime[KEY_QTY] = {0};

#if 1
const unsigned char key_table[]=
{
    KEY_SW1,
    KEY_SW2,
    KEY_SW3,
    KEY_PWR,
    KEY_CW,
    KEY_CCW
};
#else
const unsigned char key_table[]=
{
    KEY_PWR,
    KEY_UP,
    KEY_DOWN,
    KEY_ESC,
    KEY_MODE,
    KEY_OK
};
#endif
// 11: 버튼이 막 눌려짐(Push)
// 10: 버튼이 ON 상태
// 01: 버튼이 막 떨어짐(Pull)
// 00: 버튼이 OFF 상태
static void inc_key(uint8_t key) 
{ 
  uint8_t byte; 

  byte = KeyStatus[key];  	        // 버튼의 직전 상태값 가져오기 
  if ((byte & 0x3F) < ON_TIME)	byte++;	// 버튼이 눌려진 시간값을 UP count 
  if ((byte & 0x3F) >= ON_TIME)   	// 버튼이 일정시간동안 눌려져 있었다면 PUSH로 판정 
  { 
      if ((byte & 0x80) == 0)         // 버튼의 이전상태가 OFF 
      { 
            byte = byte | 0xC0;    // 버튼이 ON상태임을 표시(b7:1) 
            PushKey = key;         // [PUSH]로 인식된 버튼의 논리값을 결정 
      }
      else      // 버튼의 이전상태가 ON 
      {
           byte = byte | 0x80;
           byte = (byte & (~(uint8_t)0x40));		
      }
  }; 

  KeyStatus[key] = byte;       // 버튼의 상태값을 저장                       
} 

static void dec_key(uint8_t  key) 
{ 
  uint8_t byte; 
  byte = KeyStatus[key];                // 버튼의 직전 상태값 가져오기 
  if ((byte & 0x3F) > 0) 	byte--;       // 버튼이 눌려진 시간값을 DOWN count 
  if ((byte & 0x3F) == 0) 	      // 버튼이 일정시간동안 떨어져 있었다면 PULL로 판정 
  { 
      if ((byte & 0x80) == 0x80)          // 버튼의 이전상태가 ON
      { 
          byte = (byte & (~(uint8_t)0x80)); // 버튼 OFF
          byte = byte | 0x40;      // Up Detect
          PullKey = key;        // [PULL]로 인식된 버튼의 논리값을 결정 			
      }
      else
      {
          byte = 0;                                // 버튼이 OFF상태임을 표시(b7:0) 
      }
  };

  KeyStatus[key] = byte;                 // 버튼의 상태값을 저장    
}  


// 1개의 버튼에 대한 반복키 생성조건을 조사 눌려진 시간에 따라 키값을 생성 잠시라도 버튼을 떼면 눌려진 시간을 클리어
static void Repeat_Key_Detect(uint8_t key_no)
{
  if ((KeyStatus[key_no]&0xC0) == 0x80)                 // 버튼이 완전히 눌려져 있는지 확인
  {      
      KeyOnTime[key_no]++;
      if (KeyOnTime[key_no] == REPEAT_KEY_START)    	// 버튼이 눌려진 시간을 카운트업하고 발생시간이 되었는지 확인
      {
            RepeatKey=key_no;                           // 발생시간이 되었으면, 해당 키값을 부여
            KeyOnTime[key_no] -= CONTINUE_KEY_TIME;     // 다음 발생주기 만큼 버튼이 눌려진 시간을 감산
      }
  }
  else
  {
      KeyOnTime[key_no] = 0;       
  }	
}

void Repeat_key_Scan() 
{
  Repeat_Key_Detect(0);
  Repeat_Key_Detect(1);
  Repeat_Key_Detect(2);
  Repeat_Key_Detect(3);
  Repeat_Key_Detect(4);
  Repeat_Key_Detect(5);
}


//#define SW1_Pin GPIO_PIN_1
//#define SW1_GPIO_Port GPIOC

#define KEY_PWR_CHK           (SW1_GPIO_Port ->IDR & SW1_Pin)
 

void KeyScan (void) 
{ 
  if(KEY_PWR_CHK) dec_key(0);
  else inc_key(0);
  
} 


void InitKey (void ) 
{ 
    PullKey = KEY_NONE;
    PushKey = KEY_NONE;
    RepeatKey = KEY_NONE;
    VirtualKey = KEY_NONE;
}
