#ifndef _KEYSCAN_H_
#define _KEYSCAN_H_

#ifdef KEYSCAN_GLOBAL
  #define KEYSCAN_EXT
#else
  #define KEYSCAN_EXT extern
#endif

#define KEY_NONE	        0xFF
#if 1
#define KEY_SW1	        0x01
#define KEY_SW2 	        0x02
#define KEY_SW3 	        0x03
#define KEY_PWR                0x04
#define KEY_CW                  0x05
#define KEY_CCW                0x06
#else
#define KEY_PWR	        0x01
#define KEY_UP 	                0x02
#define KEY_DOWN 	        0x03
#define KEY_ESC                 0x04
#define KEY_MODE              0x05
#define KEY_OK                  0x06

#define KEY_PWR_LONG       0x11
#define KEY_UP_LONG	0x12
#define KEY_DOWN_LONG 	 0x13
#define KEY_ESC_LONG        0x14
#define KEY_MODE_LONG     0x15
#define KEY_OK_LONG         0x16

#endif

KEYSCAN_EXT uint8_t VirtualKey;              // PUSH 버튼의 값 
KEYSCAN_EXT uint8_t PushKey;              // PUSH 버튼의 값 
KEYSCAN_EXT uint8_t PullKey;               // PULL 버튼의 값 
KEYSCAN_EXT uint8_t RepeatKey;
KEYSCAN_EXT const uint8_t       key_table[];

KEYSCAN_EXT void KeyScan(void);
KEYSCAN_EXT void InitKey(void);
KEYSCAN_EXT void Repeat_key_Scan(void);

#endif

