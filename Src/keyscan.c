#define KEYSCAN_GLOBAL

#include "main.h"
#include "keyscan.h"


#define KEY_QTY      6      // ��ư�� �� 
#define ON_TIME       5      // ��ư ���� �νĽð� (10 x 10ms = 50ms) 
#define ON_MAX       5

#define REPEAT_KEY_START       250    //  40*12ms +40  �ݺ�Ű�� ó�� �߻��ϴ� �����Դϴ�.
#define CONTINUE_KEY_TIME      20     //  �ݺ�Ű�� �߻��ϴ� �ֱ��Դϴ�.


volatile unsigned  char  KeyStatus[KEY_QTY]=  {0};       // ��ư 1���� 1 byte�� ���۸� �Ҵ� 
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
// 11: ��ư�� �� ������(Push)
// 10: ��ư�� ON ����
// 01: ��ư�� �� ������(Pull)
// 00: ��ư�� OFF ����
static void inc_key(uint8_t key) 
{ 
  uint8_t byte; 

  byte = KeyStatus[key];  	        // ��ư�� ���� ���°� �������� 
  if ((byte & 0x3F) < ON_TIME)	byte++;	// ��ư�� ������ �ð����� UP count 
  if ((byte & 0x3F) >= ON_TIME)   	// ��ư�� �����ð����� ������ �־��ٸ� PUSH�� ���� 
  { 
      if ((byte & 0x80) == 0)         // ��ư�� �������°� OFF 
      { 
            byte = byte | 0xC0;    // ��ư�� ON�������� ǥ��(b7:1) 
            PushKey = key;         // [PUSH]�� �νĵ� ��ư�� ������ ���� 
      }
      else      // ��ư�� �������°� ON 
      {
           byte = byte | 0x80;
           byte = (byte & (~(uint8_t)0x40));		
      }
  }; 

  KeyStatus[key] = byte;       // ��ư�� ���°��� ����                       
} 

static void dec_key(uint8_t  key) 
{ 
  uint8_t byte; 
  byte = KeyStatus[key];                // ��ư�� ���� ���°� �������� 
  if ((byte & 0x3F) > 0) 	byte--;       // ��ư�� ������ �ð����� DOWN count 
  if ((byte & 0x3F) == 0) 	      // ��ư�� �����ð����� ������ �־��ٸ� PULL�� ���� 
  { 
      if ((byte & 0x80) == 0x80)          // ��ư�� �������°� ON
      { 
          byte = (byte & (~(uint8_t)0x80)); // ��ư OFF
          byte = byte | 0x40;      // Up Detect
          PullKey = key;        // [PULL]�� �νĵ� ��ư�� ������ ���� 			
      }
      else
      {
          byte = 0;                                // ��ư�� OFF�������� ǥ��(b7:0) 
      }
  };

  KeyStatus[key] = byte;                 // ��ư�� ���°��� ����    
}  


// 1���� ��ư�� ���� �ݺ�Ű ���������� ���� ������ �ð��� ���� Ű���� ���� ��ö� ��ư�� ���� ������ �ð��� Ŭ����
static void Repeat_Key_Detect(uint8_t key_no)
{
  if ((KeyStatus[key_no]&0xC0) == 0x80)                 // ��ư�� ������ ������ �ִ��� Ȯ��
  {      
      KeyOnTime[key_no]++;
      if (KeyOnTime[key_no] == REPEAT_KEY_START)    	// ��ư�� ������ �ð��� ī��Ʈ���ϰ� �߻��ð��� �Ǿ����� Ȯ��
      {
            RepeatKey=key_no;                           // �߻��ð��� �Ǿ�����, �ش� Ű���� �ο�
            KeyOnTime[key_no] -= CONTINUE_KEY_TIME;     // ���� �߻��ֱ� ��ŭ ��ư�� ������ �ð��� ����
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
