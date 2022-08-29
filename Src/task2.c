#define TASK2_GLOBAL

#include <stdio.h>    
#include <ctype.h>

#include "main.h"
#include "keyscan.h"
#include "soft_timer.h"
#include "buffer.h"
#include "ringbuffer.h"
#include "xprintf.h"

#include "task2.h"

////////////////////////////////////////////////////////////
extern TIM_HandleTypeDef htim7;
extern CONFIG_T gConfig;

extern void sound_Paly(uint8_t);

////////////////////////////////////////////////////////////

BUFFER_t USART_Buffer;
uint8_t  USARTBuffer[RX_RING_SIZE];

////////////////////////////////////////////////////////////

#define CHARISNUM(x)    ((x) >= '0' && (x) <= '9')
#define CHAR2NUM(x)     ((x) - '0')

static int32_t ParseNumber(char* ptr, uint8_t* cnt);
static void checkCmdArgs(CONFIG_T *sysconf,COMMAMD_ID id, uint8_t num);
static void cmd_hemp(void);

////////////////////////////////////////////////////////////

static char     recevied[256];


////////////////////////////////////////////////////////////
void InituserTask02(CONFIG_T *sysconf)
{
  DBG_PRINT("\r\nNS-US300 $ ");
  setSysTimeOut(DEBUG_TIMER,1);
  HAL_TIM_Base_Start_IT(&htim7);
}

void userTask02(CONFIG_T *sysconf)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  if(getSysTimeOut(DEBUG_TIMER) == 0)
  {
    uint16_t res , i ;
    COMMAMD_ID id;
    const char *text = "\r\nNS-US300 $ ";
  
    if(sysconf->state != 0)
    {
      char* ptr;
      uint16_t num = 0;
      res = BUFFER_ReadString(&USART_Buffer, recevied, sizeof(recevied));
      if(res > 0 ) {
        
        for(i = 0 ; i < res ;i++) {
          if (isupper(recevied[i])){
              recevied[i] = tolower(recevied[i]);
          }
        }
        
        if(strstr(recevied,"nsbt") != NULL) {

          /* Get token */
          ptr = strtok(recevied," ");
          
          /* Do it until token != NULL */
          while (ptr != NULL) {
            /* Get positions */
            switch (num++) {
              case 0: 
                id = CMD_unknown;
                //xprintf("");
                break;
              case 1:
                /* Ignore first and last " */
                if(strstr (ptr,"?") != NULL) {
                  id = CMD_HELP;
                } else if(strstr (ptr,"help") != NULL) {
                  id = CMD_HELP;
                } else if(strstr (ptr,"version") != NULL) {
                  id = CMD_VERSION;
                } else if(strstr(ptr,"setfrequency") != NULL)     {
                  // setFrequency
                  id = CMD_setFrequency;
                } else if(strstr(ptr,"setoutputvoltage") != NULL) {
                  //setOutputVoltage
                  id = CMD_setOutputVoltage;
                } else if(strstr(ptr,"settbdandduty")!= NULL)    {
                  //setTBDAndDuty
                  id = CMD_setTBDAndDuty;
                } else if(strstr(ptr,"settiming")!= NULL)         {
                  //setTiming
                  id = CMD_setTiming;
                } else if(strstr(ptr,"setimpedance")!= NULL)      {
                  //setImpedance
                  id = CMD_setImpedance;
                } else if(strstr(ptr,"setdelay")!= NULL)      {
                  //setImpedance
                  id = CMD_setDelay;
                } else if(strstr(ptr,"setabnormalstopmode")!= NULL) {
                  //setAbNormalStopMode
                  id = CMD_setAbnormalStopMode;
                } else if(strstr(ptr,"sonication")!= NULL) {
                  id = CMD_sonication;
                } else if(strstr(ptr,"clearerror")!= NULL) {
                  //clearError
                  id = CMD_clearError;
                } else if(strstr(ptr,"getfrequency")!= NULL)     {
                  //getFrequency
                  id = CMD_getFrequency;
                } else if(strstr(ptr,"getoutputvoltage")!= NULL) {
                  //getOutputVoltage
                  id = CMD_getOutputVoltage;
                } else if(strstr(ptr,"gettbdandduty")!= NULL)    {
                  //getTBDAndDuty
                  id = CMD_getTBDAndDuty;
                } else if(strstr(ptr,"gettiming")!= NULL)         {
                  //getTiming
                  id = CMD_getTiming;
                } else if(strstr(ptr,"getimpedance")!= NULL)      {
                  //getImpedance
                  id = CMD_getImpedance;
                } else if(strstr(ptr,"getdelay")!= NULL)      {
                  //setImpedance
                  id = CMD_getDelay;
                } else if(strstr(ptr,"getabnormalstopmode")!= NULL) {
                  //getAbnormalStopMode
                  id = CMD_getAbnormalStopMode;
                } else if(strstr(ptr,"setdigipot")!= NULL) {
                  //setDigpot
                  id = CMD_setDigipot;
                }
                break;
              case 2: 
                if(id == CMD_setFrequency) {
                  sysconf->n_setFrequency =  ParseNumber(ptr,NULL);
                } 
                else if(id == CMD_setOutputVoltage) {
                  sysconf->n_setOutputVoltage =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setTBDAndDuty) {
                  sysconf->n_setTBD =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setTiming) {
                  sysconf->n_setSD =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setImpedance) {
                  sysconf->n_setImpedance =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setDelay) {
                  sysconf->n_setDelay[RF_CH2] =  uSECTotick(ParseNumber(ptr,NULL));
                }
                else if(id == CMD_setAbnormalStopMode) {
                  sysconf->n_setAbnormalStopMode =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_sonication) {
                  if(strstr(ptr,"start") != NULL) {
                    sysconf->n_sonication = 1;

                  } else if(strstr(ptr,"stop") != NULL) {
                    sysconf->n_sonication = 0;
                  }
                }
                else if(id == CMD_setDigipot) {
                  sysconf->digipotCH =  ParseNumber(ptr,NULL);
                }
                break;
              case 3:
                if(id == CMD_setTBDAndDuty) {
                  sysconf->n_setDuty =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setTiming) {
                  sysconf->n_setISI =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setDelay) {
                  sysconf->n_setDelay[RF_CH3] =  uSECTotick(ParseNumber(ptr,NULL));
                }
                else if(id == CMD_setAbnormalStopMode) {
                  sysconf->n_setAbnormalStopMaxV =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setDigipot) {
                  sysconf->setDigipot =  ParseNumber(ptr,NULL);
                }
                break;
              case 4: 
                if(id == CMD_setTiming) {
                  sysconf->n_setTD =  ParseNumber(ptr,NULL);
                }
                else if(id == CMD_setDelay) {
                  sysconf->n_setDelay[RF_CH4] =  uSECTotick(ParseNumber(ptr,NULL));
                }
                else if(id == CMD_setAbnormalStopMode) {
                  sysconf->n_setAbnormalStopMaxI =  ParseNumber(ptr,NULL);
                }
                break;
              case 5:
                if(id == CMD_setDelay) {
                  sysconf->n_setDelay[RF_CH5] =  uSECTotick(ParseNumber(ptr,NULL));
                }
                else if(id == CMD_setAbnormalStopMode) {
                  sysconf->n_setAbnormalStopMinV =  ParseNumber(ptr,NULL);
                }
                break;
              case 6:
                if(id == CMD_setDelay) {
                  sysconf->n_setDelay[RF_CH6] =  uSECTotick(ParseNumber(ptr,NULL));
                }
                else if(id == CMD_setAbnormalStopMode) {
                  sysconf->n_setAbnormalStopMinI =  ParseNumber(ptr,NULL);
                }
                break;
              // case 7:
              //   if(id == CMD_setDelay) {
              //     sysconf->n_setDelay[RF_CH7] =  from100uSEC(ParseNumber(ptr,NULL));
              //   }
              //   break;  
              default: break;
            }

            /* Get new token */
            ptr = strtok(NULL, " ");
          }
          //
          checkCmdArgs(sysconf,id,num);
          displayEnv(sysconf,id);
        }
        
        xprintf(text);
      }
    }

    setSysTimeOut(DEBUG_TIMER,1);
  }
  /* USER CODE END StartTask02 */
}



static int32_t ParseNumber(char* ptr, uint8_t* cnt) {
	uint8_t minus = 0;
	int32_t sum = 0;
	uint8_t i = 0;
	
	/* Check for minus character */
	if (*ptr == '-') {
		minus = 1;
		ptr++;
		i++;
	}
	
	/* Parse number */
	while (CHARISNUM(*ptr)) {
		sum = 10 * sum + CHAR2NUM(*ptr);
		ptr++;
		i++;
	}
	
	/* Save number of characters used for number */
	if (cnt != NULL) {
		*cnt = i;
	}
	
	/* Minus detected */
	if (minus) {
		return 0 - sum;
	}
	
	/* Return number */
	return sum;
}

static void checkCmdArgs(CONFIG_T *sysconf,COMMAMD_ID id, uint8_t num)
{

  uint32_t error = 0;
  switch(id) {
   case CMD_setFrequency:
    if(num == 3) {
      sysconf->updateEnv |= (1 << CMD_setFrequency) ;
      xprintf("\r\nsetFrequency OK");
    } else {
      sysconf->n_setFrequency = sysconf->setFrequency;
      if(num > 3)
        DBG_ERROR("\r\nsetFrequency too many arguments");
      else
        DBG_ERROR("\r\nsetFrequency too few arguments");
    }
    break;
   case CMD_setOutputVoltage:
    if(num == 3) {
      sysconf->updateEnv |= (1 << CMD_setOutputVoltage) ;
      xprintf("\r\nsetOutputVoltage OK");
    } else {
      sysconf->n_setOutputVoltage = sysconf->setOutputVoltage;
      if(num > 3)
        DBG_ERROR("\r\nsetOutputVoltage too many arguments");
      else
        DBG_ERROR("\r\nsetOutputVoltage too few arguments");
    }
    break;
   case CMD_setTBDAndDuty:
    if(num == 4) {
      sysconf->updateEnv |= (1 << CMD_setTBDAndDuty) ;
      xprintf("\r\nsetTBDAndDuty OK");
    } else {
      sysconf->n_setTBD  =  sysconf->setTBD;
      sysconf->n_setDuty = sysconf->setDuty;
      if(num > 4)
        DBG_ERROR("\r\nsetTBDAndDuty too many arguments");
      else
        DBG_ERROR("\r\nsetTBDAndDuty too few arguments");
    }
    break;
   case CMD_setTiming:
    if(num == 5) {
      if(sysconf->n_setSD <  1 || sysconf->n_setSD > 500)
        error |= (1<< 0);
      if(sysconf->n_setISI < 0 || sysconf->n_setISI > 30)
        error |= (1<< 1);
      if(sysconf->n_setTD < 0  || sysconf->n_setTD > 1800)
        error |= (1<< 2);
      if(!error) {  
        sysconf->updateEnv |= (1 << CMD_setTiming) ;
        xprintf("\r\nsetTiming OK");
      } else {
        if(error & (1<<0))
          DBG_ERROR("\r\nFail SD is must 1~500 !!!");
        if(error & (1<<1))
          DBG_ERROR("\r\nFail ISI is must 0~30 !!!");
        if(error & (1<<2))
          DBG_ERROR("\r\nFail TD is must 0~1800 !!!");
      }
    } else {
      sysconf->n_setSD  = sysconf->setSD;
      sysconf->n_setISI = sysconf->setISI;
      sysconf->n_setTD  = sysconf->setTD;
      if(num > 5)
        DBG_ERROR("\r\nsetTiming too many arguments");
      else
        DBG_ERROR("\r\nsetTiming too few arguments");
    }
    break;
   case CMD_setImpedance:
    if(num == 3) {
      sysconf->updateEnv |= (1 << CMD_setImpedance) ;
      xprintf("\r\nsetImpedance OK");
    } else {
      sysconf->n_setImpedance = sysconf->setImpedance;
      if(num > 3)
        DBG_ERROR("\r\nsetImpedance too many arguments");
      else
        DBG_ERROR("\r\nsetImpedance too few arguments");
    }
    break;
   case CMD_setDelay:
    if(num == 5) {
      sysconf->updateEnv |= (1 << CMD_setDelay) ;
      xprintf("\r\nsetDelay OK");
    } else {
      //sysconf->n_setDelay[RF_CH0] = sysconf->setDelay[RF_CH0];
      //sysconf->n_setDelay[RF_CH1] = sysconf->setDelay[RF_CH1];
      sysconf->n_setDelay[RF_CH2] = sysconf->setDelay[RF_CH2];
      sysconf->n_setDelay[RF_CH3] = sysconf->setDelay[RF_CH3];
      sysconf->n_setDelay[RF_CH4] = sysconf->setDelay[RF_CH4];
      //sysconf->n_setDelay[RF_CH5] = sysconf->setDelay[RF_CH5];
      //sysconf->n_setDelay[RF_CH6] = sysconf->setDelay[RF_CH6];
      
      if(num > 5)
        DBG_ERROR("\r\nsetsetDelay too many arguments");
      else
        DBG_ERROR("\r\nsetsetDelay too few arguments");
    }
    break;
   case CMD_setAbnormalStopMode:
    if(num == 7) {
      sysconf->updateEnv |= (1 << CMD_setAbnormalStopMode) ;
      xprintf("\r\nsetAbnormalStopMode OK");
    } else {
      sysconf->n_setAbnormalStopMode = sysconf->setAbnormalStopMode;
      sysconf->n_setAbnormalStopMaxI = sysconf->setAbnormalStopMaxI;
      sysconf->n_setAbnormalStopMaxV = sysconf->setAbnormalStopMaxV;
      sysconf->n_setAbnormalStopMinV = sysconf->setAbnormalStopMinV;
      sysconf->n_setAbnormalStopMinI = sysconf->setAbnormalStopMinI;

      if(num > 7)
        DBG_ERROR("\r\nsetAbnormalStopMode too many arguments");
      else
        DBG_ERROR("\r\nsetAbnormalStopMode too few arguments");
    }
    break;
   case CMD_sonication:
    if(num == 2) {
      INFO("\r\nsonication [%s]\r\n",sysconf->sonication == 1? "start":"stop");
    } else if(num == 3) {
      sysconf->updateEnv |= (1 << CMD_sonication) ;
      xprintf("\r\nsonication OK");
      //xprintf("\r\n %s sonication",sysconf->n_sonication == 1? "start":"stop");
    } else {
      sysconf->n_sonication = sysconf->sonication;
      DBG_ERROR("\r\nsonication too many arguments");
    }
    break;
   case CMD_setDigipot:
    if(num == 4) {
      if(sysconf->digipotCH == 1 || sysconf->digipotCH == 2 || sysconf->digipotCH == 3 || sysconf->digipotCH == 4)
      {
        sysconf->updateEnv |= (1 << CMD_setDigipot) ;
      } else {
        DBG_ERROR("\r\nsetDigipot Invalid CH");
      }
    } else {
      DBG_ERROR("\r\nsetDigipot Invalid arguments");
    }
    break;
   case CMD_clearError:
      xprintf("\r\nclearError OK");
    break;
  }
}

static void cmd_hemp(void)
{
  INFO("\r\n=================================");
  INFO("\r\nCommand List");
  INFO("\r\n%s","nsbt version");
  INFO("\r\n%s","nsbt setFrequency 100");
  INFO("\r\n%s","nsbt setTBDAndDuty 10 20");
  INFO("\r\n%s","nsbt setTiming 100 3 600");
  INFO("\r\n%s","nsbt setImpedance 2");
  INFO("\r\n%s","nsbt setAbnormalStopMode 1 1200 1100 300 200");
  INFO("\r\n%s","nsbt sonication start[stop]");
  INFO("\r\n%s","nsbt clearError");
  INFO("\r\n%s","nsbt setDigipot 0 25");
  INFO("\r\n=================================");
}

void displayEnv(CONFIG_T *sysconf, COMMAMD_ID id)
{
  char str[64];
  switch(id) {
   case CMD_HELP:
      cmd_hemp();
      break;  
   case CMD_VERSION:
      display_version();
      break; 
   case CMD_getFrequency:
      INFO("\r\nFrequency = %d Hz\r\n",sysconf->setFrequency);
      break;
   case CMD_getOutputVoltage:
      INFO("\r\nOutputVoltage = %d V\r\n",sysconf->setOutputVoltage);
      INFO("\r\n");
      break;
   case CMD_getTBDAndDuty:
      sprintf(str,"\r\nTBD  = %0.1f ms (%d) ",(float)to1mSEC((float)sysconf->setTBD),sysconf->setTBD);
      INFO(str);
      sprintf(str,"\r\nRPR  = %0.3f ms (%d) ",(float)to1mSEC((float)sysconf->setPRP),sysconf->setPRP);
      INFO(str);
      INFO("\r\nDuty = %d %% \r\n",sysconf->setDuty);
      INFO("\r\n");
      break;
   case CMD_getTiming:
      INFO("\r\nSD  = %6d ms (%8d)",    to1mSEC(sysconf->setSD),  sysconf->setSD);
      INFO("\r\nISI = %6d S  (%8d)",    to1sSEC(sysconf->setISI), sysconf->setISI);
      INFO("\r\nBI  = %6d S  (%8d)",    to1sSEC(sysconf->setBI),  sysconf->setBI);
      INFO("\r\nTD  = %6d S  (%8d)\r\n",to1sSEC(sysconf->setTD),  sysconf->setTD);
      break;
   case CMD_getDelay:
      INFO("\r\nCH1  = %6d us (%8d)",    sysconf->setDelay[RF_CH1]*TOUSEC , sysconf->setDelay[RF_CH1]);
      INFO("\r\nCH2  = %6d us (%8d)",    sysconf->setDelay[RF_CH2]*TOUSEC , sysconf->setDelay[RF_CH2]);
      INFO("\r\nCH3  = %6d us (%8d)",    sysconf->setDelay[RF_CH3]*TOUSEC,  sysconf->setDelay[RF_CH3]);
      INFO("\r\nCH4  = %6d us (%8d)\r\n",sysconf->setDelay[RF_CH4]*TOUSEC,  sysconf->setDelay[RF_CH4]);
      break;
   case CMD_getImpedance:
      INFO("\r\nImpedance  = %d \r\n",sysconf->setImpedance);
      break;
   case CMD_getAbnormalStopMode:
      INFO("\r\nAbnormal[%3s]",sysconf->setAbnormalStopMode == 0? "Off":"On");
      INFO("\r\nMax Volt     = %5d ",sysconf->setAbnormalStopMaxV);
      INFO("\r\nmin Volt     = %5d ",sysconf->setAbnormalStopMinV);
      INFO("\r\nMax Current  = %5d ",sysconf->setAbnormalStopMaxI);
      INFO("\r\nmin Current  = %5d \r\n",sysconf->setAbnormalStopMinI);
      break;
  }
}
