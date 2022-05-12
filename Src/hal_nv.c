

#include "main.h"
#include "xprintf.h"
#include "hal_nv.h"


// #define USE_INTERNAL_FLASH
/*********************************************************************
* MACROS
*/


//#define FLASH_LOG_ELEMENT_SIZE  16
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* CONSTANTS
*/
#define NV_FLASH_INIT_FLAG_LEN		4
#define	NV_ID_FREQUENCY_LEN				4
#define NV_ID_OUTPUTVOLTE					4
#define NV_ID_TBD					        4
#define NV_ID_DUTY					      4
#define NV_ID_PRP					        4
#define NV_ID_SD				          4
#define NV_ID_ISI						      4
#define NV_ID_BI				          4
#define NV_ID_TD				          4
#define NV_ID_IMPEDANCE						4
#define NV_ID_ABNORMAL_STOPMODE		4

#define NV_ID_ABNORMAL_STOPMAX_V	4
#define NV_ID_ABNORMAL_STOPMIN_V	4
#define NV_ID_ABNORMAL_STOPMAX_I	4
#define NV_ID_ABNORMAL_STOPMIN_I	4

#define NV_ID_DELAY_CH			      (4*7)

#define	NV_ID_SERIAL_NUMBER_LEN		28

static uint16_t Nv_Idx[NV_ID_MAX_NUM] = {
  NV_FLASH_INIT_FLAG_LEN,
  NV_ID_FREQUENCY_LEN,
  NV_ID_OUTPUTVOLTE_LEN,
  NV_ID_TBD_LEN,
  NV_ID_DUTY_LEN,
  NV_ID_PRP_LEN,
  NV_ID_SD_LEN,
  NV_ID_ISI_LEN,
  NV_ID_BI_LEN,
  NV_ID_TD_LEN,
  NV_ID_IMPEDANCE_LEN,
  NV_ID_ABNORMAL_STOPMODE_LEN,
  NV_ID_ABNORMAL_STOPMAX_V_LEN,
  NV_ID_ABNORMAL_STOPMIN_V_LEN,
  NV_ID_ABNORMAL_STOPMAX_I_LEN,
  NV_ID_ABNORMAL_STOPMIN_I_LEN,
  NV_ID_DELAY_CH_LEN
};

#ifdef STM32F103xx
#define HAL_FLASH_PAGE_ADDR_127   ((uint32_t)0x0803F800) /* Base @ of Page 127, 2 Kbytes */
#define HAL_FLASH_PAGE_ADDR_126   ((uint32_t)0x0803F000) /* Base @ of Page 126, 2 Kbytes */
#define HAL_FLASH_PAGE_ADDR_125   ((uint32_t)0x0803E800) /* Base @ of Page 125, 2 Kbytes */
#define HAL_FLASH_PAGE_ADDR_124   ((uint32_t)0x0803E000) /* Base @ of Page 124, 2 Kbytes */
#define HAL_FLASH_PAGE_ADDR_123   ((uint32_t)0x0803D800) /* Base @ of Page 123, 2 Kbytes */
#define HAL_FLASH_PAGE_ADDR_122   ((uint32_t)0x0803D000) /* Base @ of Page 122, 2 Kbytes */
#define HAL_FLASH_PAGE_ADDR_121   ((uint32_t)0x0803C800) /* Base @ of Page 121, 2 Kbytes */

#define MAX_NV_IDS                 1024

// DAC MID data : change to manufacturing data
#define	NV_ID_DEV_ADDRESS			    HAL_FLASH_PAGE_ADDR_127
// dev config information : 
#define NV_ID_TABLE1_ADDRESS		  HAL_FLASH_PAGE_ADDR_126
// serial number
#define NV_ID_SERIAL_NO_ADDRESS		HAL_FLASH_PAGE_ADDR_125
// RTC data
#define NV_ID_RTC_DATA_ADDRESS		HAL_FLASH_PAGE_ADDR_124

#define USER_SPACE_OFFSET         1024 // id > 100

#define	FLASH_SECTOR_SIZE			    4096
#endif

#ifdef STM32F205xx
#define HAL_FLASH_SECTOR_5   ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define HAL_FLASH_SECTOR_4   ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define HAL_FLASH_SECTOR_3   ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define HAL_FLASH_SECTOR_2   ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define HAL_FLASH_SECTOR_1   ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define HAL_FLASH_SECTOR_0   ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */

#define MAX_NV_IDS                 1024

// DAC MID data : change to manufacturing data
#define	NV_ID_DEV_ADDRESS			    HAL_FLASH_SECTOR_4
// dev config information : 
#define NV_ID_TABLE1_ADDRESS		  HAL_FLASH_SECTOR_4
// serial number
#define NV_ID_SERIAL_NO_ADDRESS		HAL_FLASH_SECTOR_4
// RTC data
#define NV_ID_RTC_DATA_ADDRESS		HAL_FLASH_SECTOR_4

#define USER_SPACE_OFFSET         1024 // id > 100

#define	FLASH_SECTOR_SIZE			    0x20000


#endif

/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/
#if 0
// SST25VF016B.c
extern void Byte_Program(unsigned long Dst, unsigned char byte);
// 256 byte page program
extern void Page_Program(unsigned long Dst, char *buf, int len);
extern void Read_Cont(unsigned long Dst, unsigned long no_bytes, uint8_t destbuf[]);
extern unsigned char Read(unsigned long Dst); 
extern void Sector_Erase(unsigned long Dst); 
extern unsigned char Read_Status_Register(void);
extern unsigned char Read_Config_Register(void);
#endif

/*********************************************************************
* LOCAL VARIABLES
*/

// 4096/16 =  256 byte reserved
// #define	USE_NV_MAX_SIZE	(FLASH_SECTOR_SIZE/16)	

// 65536/64 =  256 byte reserved
#define	USE_NV_MAX_SIZE	(128)	

static uint8_t tmp_buf[USE_NV_MAX_SIZE];

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void NvUpdate(uint32_t addr, NvIdHdr_t *pIdt, uint8_t *pData);
/*
 * Initialize NV service
 */
void NvInit( void *p )
{
  UNUSED(p);
}

static void *Memcpy( void *dst, const void *src, unsigned int len )
{
  const uint32_t *pSrc;
  uint32_t *pDst;

  pSrc = src;
  pDst = dst;

  while (len) {
          *pDst = *(__IO uint32_t *)pSrc;
          pDst++; pSrc++;
          len -= 4;
  }

  return ( pDst );
}

void HalFlashWrite(uint32_t address, uint8_t *buf, uint16_t cnt)
{
	uint32_t tmp;
	int len = cnt;

        if (console_debug) {
          //DBG_INFO("%s: len = %d\n", __func__, len);
          xprintf("%s: len = %d\n", __func__, len);
        }

	HAL_FLASH_Unlock();
	
	while (len > 0)
	{
		{
			memcpy(&tmp, buf, 4);
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, tmp) != HAL_OK)
			{
				INFO("%s: HAL_FLASH_Program error.\n", __FUNCTION__);
				HAL_FLASH_Lock();
				return;
			}
			address = address + 4;
			buf = buf+4;
			len = len - 4;
		}
	}

	HAL_FLASH_Lock();

  if (console_debug) {
    INFO("%s: done.\n", __FUNCTION__);
  }

}


int HalFlashErase(uint32_t pgAddr)
{

	uint32_t PageError1 = 0;
  uint32_t addr = pgAddr;

	FLASH_EraseInitTypeDef EraseInitStruct;

  if (console_debug) {
    INFO("%s: addr = 0x%x\n", __FUNCTION__, pgAddr);
  }

#ifdef STM32F205xx
//  for(int i = 0; i < 6 ; i++) {
//    if(FLASH_SECTOR_TABLE[i] == addr)
//    {
//      addr = i;
//      break;
//    }
//  }
#endif
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS ;//FLASH_TYPEERASE_PAGES;
	//EraseInitStruct.PageAddress = pgAddr/0x800;
	EraseInitStruct.Sector = 4;
	EraseInitStruct.NbSectors     = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError1) != HAL_OK)
	{
		ERROR("%s: HAL_FLASHEx_Erase error (%d).\n", __FUNCTION__,PageError1);
		HAL_FLASH_Lock();
		return -1;
	}	

	HAL_FLASH_Lock();
	return 0;
}

void NvUpdate(uint32_t addr, NvIdHdr_t *pIdt, uint8_t *pData)
{
  INFO("%s: pIdt = 0x%x, addr = 0x%x\n", __FUNCTION__, pIdt, addr);

  if(pIdt == NULL)
  {
    uint8_t *tmp =NULL , *p = NULL;
    uint32_t pg = NV_ID_TABLE1_ADDRESS;
    uint16_t id=addr;
	  int i;

	  tmp = tmp_buf;

    INFO("%s: size = 0x%x (%d)\n", __func__, USE_NV_MAX_SIZE, USE_NV_MAX_SIZE);

	  Memcpy((uint8_t*)tmp , (uint8_t*)pg, USE_NV_MAX_SIZE);

    HalFlashErase(pg);

    //update memory
    p = tmp;
    for(i=0;i<id;i++)
    {
      p += Nv_Idx[i];
    }
    Memcpy(p,pData,Nv_Idx[id]);

    HalFlashWrite(pg, (uint8_t*)tmp, USE_NV_MAX_SIZE);
  }
  else
  {
	  WARRING("%s: warning !!!!! pIdt != NULL.\n", __func__);
  }
}

void NvErase(void)
{
  HalFlashErase(NV_ID_TABLE1_ADDRESS);
}

/**
  Read, or initialize
 */
void NvReadOrInit(uint16_t nvID, uint16_t len, void * pDstVal, const void* pInitVal )
{
  int status = NvRead(nvID, len, pDstVal);

  if(status != NV_SUCCESS || memcmp(pDstVal, pInitVal, len) != 0)
  {
    Memcpy(pDstVal, pInitVal, len);
    //All values are 0xFF then it has never been configured.
    NvWrite(nvID, len, pDstVal);
  }
}

/*
 * Read an NV attribute
 */
int NvRead( uint16_t id, uint16_t len, void *buf )
{
  //NvIdHdr_t idt;
  uint32_t address, i;

  address = NV_ID_TABLE1_ADDRESS;

  if(id < NV_ID_MAX_NUM)
  {
    address = NV_ID_TABLE1_ADDRESS;
    for(i=0;i<id;i++)
    {
      address += Nv_Idx[i];
    }
    Memcpy(buf,(void*)address,Nv_Idx[id]);
  }
  else
  {
	  WARRING("%s: warning !!!!!! (id >= NV_ID_MAX_NUM).\n", __func__);
  }

  return NV_SUCCESS;
}

static int WriteValToMem(uint8_t * mem ,uint16_t id, uint16_t len, void *buf )
{
  uint32_t address;

  if(id < NV_ID_MAX_NUM)
  {
    address = 0;
    for(int i=0;i<id;i++)
    {
      address += Nv_Idx[i];
    }
    Memcpy(mem+address,buf,Nv_Idx[id]);
  }

  return NV_SUCCESS;
}

/*
 * Write an NV attribute
 */
int NvWrite( uint16_t id, uint16_t len, void *buf )
{
  uint32_t address;

  if(id < NV_ID_MAX_NUM)
  {

    address = NV_ID_TABLE1_ADDRESS;

    for(int i=0;i<id;i++)
    {
      address += Nv_Idx[i];
    }

    if(memcmp((const char*)address,(const char*)buf,Nv_Idx[id]) != 0)
    {
      NvUpdate((uint32_t)id,NULL, buf);
    }
  }
  else
  {
	  WARRING("%s: warning !!!!!! (id >= NV_ID_MAX_NUM).\n", __func__);
  }
  return NV_SUCCESS;
}

#if defined(CONFIG_SERIAL)
int NvSerialNoWrite(void *buf,uint16_t len)
{
  uint32_t address;

  address = NV_ID_SERIAL_NO_ADDRESS;
  DBG_INFO("%s: len = %d, NV_ID_SERIAL_NUMBER_LEN = %d\n", __func__, len, NV_ID_SERIAL_NUMBER_LEN);

  if(len != NV_ID_SERIAL_NUMBER_LEN)
    return YEInvalidParameter;

  if(memcmp((const char*)address,(const char*)buf,NV_ID_SERIAL_NUMBER_LEN) != 0)
  {
    DBG_INFO("%s: write data\n", __FUNCTION__);
    //uint8_t *tmp =NULL;
    uint32_t pg = NV_ID_SERIAL_NO_ADDRESS;
    HalFlashErase(pg);
    HalFlashWrite(pg, (uint8_t*)buf, NV_ID_SERIAL_NUMBER_LEN);
  }
  return NV_SUCCESS;
}

int NvSerialNoRead(uint8_t *buf)
{
  uint32_t address;

  if(buf==NULL)
    return YEMemUnavailable;

//DBG_INFO("%s: called.\n", __func__);

  address = NV_ID_SERIAL_NO_ADDRESS;

  DBG_INFO("%s: address = 0x%x, len = %d.\n", __func__, address, NV_ID_SERIAL_NUMBER_LEN);
  Memcpy(buf,(void*)address,NV_ID_SERIAL_NUMBER_LEN);

  if((buf[0])==0xFF && (buf[1])==0xFF)
    return YEInvalidState;
  else if((buf[0])==0x00 && (buf[1])==0x00)
    return YEInvalidState;
  else
    return NV_SUCCESS;
}
#endif
/*
 * Get the length of an NV item.
 */
uint16_t NvItemLen( uint16_t id )
{
  NvIdHdr_t idt;
  uint32_t address,i ;

  if(id == 0 || id == 0xffff)
    return EINVAL;

  address = NV_ID_TABLE1_ADDRESS;

  for( i = 0 ; i < 1024/*FLASH_PAGE_SIZE*/; )
  {
    idt = *(__IO NvIdHdr_t *)address;
    if(idt.id == id) {
      break;
    }
    address += NVID_HDR_LEN;
    address += idt.lens;
  }

  if( idt.id != id)
    return 0;

  return idt.lens;
}

int NvWriteDefault(void)
{
  uint8_t *tmp =NULL;
  uint32_t pg = NV_ID_TABLE1_ADDRESS;
  uint32_t val;

  int32_t array[MAX_RF_CH];

  INFO("%s: NvWrite default\n", __FUNCTION__);

	tmp = tmp_buf;

  ////////////////////////////////////////////////////////////////////
  memset(tmp,0,USE_NV_MAX_SIZE);
  HalFlashErase(pg);

  INFO("%s: after flash erase\n", __FUNCTION__);

  ////////////////////////////////////////////////////////////////////
  val = RF_DEF_FREQUENCY;
  WriteValToMem(tmp,NV_ID_FREQUENCY,NV_ID_FREQUENCY_LEN,&val);

  val = RF_DEF_OUTVOLT;
  WriteValToMem(tmp,NV_ID_OUTPUTVOLTE,NV_ID_OUTPUTVOLTE_LEN,&val);

  val = RF_DEF_TBD;
  WriteValToMem(tmp,NV_ID_TBD,NV_ID_TBD_LEN,&val);

  val = RF_DEF_DUTY;
  WriteValToMem(tmp,NV_ID_DUTY,NV_ID_DUTY_LEN,&val);

  val = RF_DEF_RPR;
  WriteValToMem(tmp,NV_ID_PRP,NV_ID_PRP_LEN,&val);

  val = RF_DEF_SD;
  WriteValToMem(tmp,NV_ID_SD,NV_ID_SD_LEN,&val);

  val = RF_DEF_ISI;
  WriteValToMem(tmp,NV_ID_ISI,NV_ID_ISI_LEN,&val);

  val = RF_DEF_BI;
  WriteValToMem(tmp,NV_ID_BI,NV_ID_BI_LEN,&val);

  val = RF_DEF_TD;
  WriteValToMem(tmp,NV_ID_TD,NV_ID_TD_LEN,&val);

  val = RF_DEF_IMPEDANCE;
  WriteValToMem(tmp,NV_ID_IMPEDANCE,NV_ID_IMPEDANCE_LEN,&val);

  val = RF_DEF_ABNORMAL_MODE;
  WriteValToMem(tmp,NV_ID_ABNORMAL_STOPMODE,NV_ID_ABNORMAL_STOPMODE_LEN,&val);

  val = RF_DEF_ABNORMAL_MAXV;
  WriteValToMem(tmp,NV_ID_ABNORMAL_STOPMAX_V,NV_ID_ABNORMAL_STOPMAX_V_LEN,&val);

  val = RF_DEF_ABNORMAL_MINV;
  WriteValToMem(tmp,NV_ID_ABNORMAL_STOPMIN_V,NV_ID_ABNORMAL_STOPMIN_V_LEN,&val);

  val = RF_DEF_ABNORMAL_MAXI;
  WriteValToMem(tmp,NV_ID_ABNORMAL_STOPMAX_I,NV_ID_ABNORMAL_STOPMAX_I_LEN,&val);

  val = RF_DEF_ABNORMAL_MINI;
  WriteValToMem(tmp,NV_ID_ABNORMAL_STOPMIN_I,NV_ID_ABNORMAL_STOPMIN_I_LEN,&val);

  memset(array, 0, sizeof(array));

  array[RF_CH0] = RF_DEF_DELAY_CH0;
  array[RF_CH1] = RF_DEF_DELAY_CH1;
  array[RF_CH2] = RF_DEF_DELAY_CH2;
  array[RF_CH3] = RF_DEF_DELAY_CH3;
  array[RF_CH4] = RF_DEF_DELAY_CH4;
  array[RF_CH5] = RF_DEF_DELAY_CH5;
  array[RF_CH6] = RF_DEF_DELAY_CH6;

  WriteValToMem(tmp,NV_ID_DELAY_CH,NV_ID_DELAY_CH_LEN,array);

  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////
  HalFlashWrite(pg, (uint8_t*)tmp, USE_NV_MAX_SIZE);

  INFO("%s: end\n", __FUNCTION__);
	return 0;
}

int NvCheckAndReset(void)
{
  uint16_t flash_flag;

  NvRead(NV_FLASH_INIT_FLAG,NV_FLASH_INIT_FLAG_LEN,&flash_flag);

  INFO("%s: flash_flag = 0x%x.\n", __FUNCTION__, flash_flag);

  if(flash_flag != 0) //true:need initialize user flash space
  {
    NvWriteDefault();
  }

	return 0;
}

void NvReadAll(void *env)
{
  int status;
  uint32_t val;
  uint16_t flash_flag;
  int32_t array[MAX_RF_CH];
  CONFIG_T *conf = (CONFIG_T *)env;


  NvRead(NV_FLASH_INIT_FLAG,NV_FLASH_INIT_FLAG_LEN,&flash_flag);
  INFO("%s: flash_flag = 0x%x.\n", __FUNCTION__, flash_flag);

  if(!conf) return;

  if(flash_flag != 0) {
    INFO("%s: WARNING !!!!!, flash memory is stange.\n", __func__);
    INFO("%s: apply default value.\n", __func__);

    conf->setFrequency      = RF_DEF_FREQUENCY;
    conf->setOutputVoltage  = RF_DEF_OUTVOLT;

    conf->setTBD  =  RF_DEF_TBD; // 25.0ms  range 0.1 ~ 50ms
    conf->setDuty =  RF_DEF_DUTY;   // 50 %    range 1 ~ 100 %
    conf->setPRP  =  RF_DEF_RPR;

    conf->setSD  =  RF_DEF_SD;      // ms
    conf->setISI =  RF_DEF_ISI;       // s
    conf->setBI  =  RF_DEF_BI;    // s 
    conf->setTD  =  RF_DEF_TD;     // s 

    conf->setImpedance = RF_DEF_IMPEDANCE;
    
    conf->setAbnormalStopMode = RF_DEF_ABNORMAL_MODE;
    conf->setAbnormalStopMaxV = RF_DEF_ABNORMAL_MAXV;
    conf->setAbnormalStopMaxI = RF_DEF_ABNORMAL_MINV;
    conf->setAbnormalStopMinV = RF_DEF_ABNORMAL_MAXI;
    conf->setAbnormalStopMinI = RF_DEF_ABNORMAL_MINI;

    conf->setDelay[RF_CH0] = RF_DEF_DELAY_CH0;
    conf->setDelay[RF_CH1] = RF_DEF_DELAY_CH1;
    conf->setDelay[RF_CH2] = RF_DEF_DELAY_CH2;
    conf->setDelay[RF_CH3] = RF_DEF_DELAY_CH3;
    conf->setDelay[RF_CH4] = RF_DEF_DELAY_CH4;
    conf->setDelay[RF_CH5] = RF_DEF_DELAY_CH5;
    conf->setDelay[RF_CH6] = RF_DEF_DELAY_CH6;
    conf->sonication = 0;
  }
  else {
    //Frequency
	  status = NvRead(NV_ID_FREQUENCY, NV_ID_FREQUENCY_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_FREQUENCY;
      NvWrite(NV_ID_FREQUENCY,NV_ID_FREQUENCY_LEN,&val);
	  }
	  conf->setFrequency = (int32_t)val;
    INFO("Frequency = %d\n", conf->setFrequency);

    //OutputVoltage
	  status = NvRead(NV_ID_OUTPUTVOLTE, NV_ID_OUTPUTVOLTE_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_OUTVOLT;
      NvWrite(NV_ID_OUTPUTVOLTE,NV_ID_OUTPUTVOLTE_LEN,&val);
	  }
	  conf->setOutputVoltage = (int32_t)val;
    INFO("OutputVoltage = %d\n", conf->setOutputVoltage);

    //TBD
	  status = NvRead(NV_ID_TBD, NV_ID_TBD_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_TBD;
      NvWrite(NV_ID_TBD,NV_ID_TBD_LEN,&val);
	  }
    conf->setTBD = (int32_t)val;
    INFO("TBD = %d\n", conf->setTBD);

    //DUTY
	  status = NvRead(NV_ID_DUTY, NV_ID_DUTY_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_DUTY;
      NvWrite(NV_ID_DUTY,NV_ID_DUTY_LEN,&val);
	  }
	  conf->setDuty = (int32_t)val;
    INFO("Duty = %d\n", conf->setDuty);

    //PRP
    status = NvRead(NV_ID_PRP, NV_ID_PRP_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_RPR;
      NvWrite(NV_ID_PRP,NV_ID_PRP_LEN,&val);
	  }
	  conf->setPRP = (int32_t)val;
    INFO("PRP = %d\n", conf->setPRP);

    //SD
    status = NvRead(NV_ID_SD, NV_ID_SD_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_SD;
      NvWrite(NV_ID_SD,NV_ID_SD_LEN,&val);
	  }
	  conf->setSD = (int32_t)val;
    INFO("SD = %d\n", conf->setSD);
    
    //ISI
    status = NvRead(NV_ID_ISI, NV_ID_ISI_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_ISI;
      NvWrite(NV_ID_ISI,NV_ID_ISI_LEN,&val);
	  }
	  conf->setISI = (int32_t)val;
    INFO("ISI = %d\n", conf->setISI);
    
    //BI
    status = NvRead(NV_ID_BI, NV_ID_BI_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_BI;
      NvWrite(NV_ID_BI,NV_ID_BI_LEN,&val);
	  }
	  conf->setBI = (int32_t)val;
    INFO("BI = %d\n", conf->setBI);

    //TD
    status = NvRead(NV_ID_TD, NV_ID_TD_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_TD;
      NvWrite(NV_ID_TD,NV_ID_TD_LEN,&val);
	  }
	  conf->setTD = (int32_t)val;
    INFO("TD = %d\n", conf->setTD);

    //IMPEDANCE
    status = NvRead(NV_ID_IMPEDANCE, NV_ID_IMPEDANCE_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_IMPEDANCE;
      NvWrite(NV_ID_IMPEDANCE,NV_ID_IMPEDANCE_LEN,&val);
	  }
	  conf->setImpedance = (int32_t)val;
    INFO("Impedance = %d\n", conf->setImpedance);

    //AbnormalStop
    status = NvRead(NV_ID_ABNORMAL_STOPMODE, NV_ID_ABNORMAL_STOPMODE_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_ABNORMAL_MODE;
      NvWrite(NV_ID_ABNORMAL_STOPMODE,NV_ID_ABNORMAL_STOPMODE_LEN,&val);
	  }
	  conf->setAbnormalStopMode = (int32_t)val;
    INFO("AbnormalStopMode = %d\n", conf->setAbnormalStopMode);

    //AbnormalStop Max V
    status = NvRead(NV_ID_ABNORMAL_STOPMAX_V, NV_ID_ABNORMAL_STOPMAX_V_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_ABNORMAL_MAXV;
      NvWrite(NV_ID_ABNORMAL_STOPMAX_V,NV_ID_ABNORMAL_STOPMAX_V_LEN,&val);
	  }
	  conf->setAbnormalStopMaxV = (int32_t)val;
    INFO("AbnormalStop Max Vout = %d\n", conf->setAbnormalStopMaxV);


    //AbnormalStop Min V
    status = NvRead(NV_ID_ABNORMAL_STOPMIN_V, NV_ID_ABNORMAL_STOPMIN_V_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_ABNORMAL_MINV;
      NvWrite(NV_ID_ABNORMAL_STOPMIN_V,NV_ID_ABNORMAL_STOPMAX_V_LEN,&val);
	  }
	  conf->setAbnormalStopMaxV = (int32_t)val;
    INFO("AbnormalStop Min Vout = %d\n", conf->setAbnormalStopMaxV);

    //AbnormalStop Max I
    status = NvRead(NV_ID_ABNORMAL_STOPMAX_I, NV_ID_ABNORMAL_STOPMAX_I_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_ABNORMAL_MAXI;
      NvWrite(NV_ID_ABNORMAL_STOPMAX_I,NV_ID_ABNORMAL_STOPMAX_I_LEN,&val);
	  }
	  conf->setAbnormalStopMaxI = (int32_t)val;
    INFO("AbnormalStop Min Vout = %d\n", conf->setAbnormalStopMaxI);

    //AbnormalStop Min I
    status = NvRead(NV_ID_ABNORMAL_STOPMIN_I, NV_ID_ABNORMAL_STOPMIN_I_LEN, (void*)&val);
	  if(status != YSuccess)
	  {
      val = RF_DEF_ABNORMAL_MINI;
      NvWrite(NV_ID_ABNORMAL_STOPMIN_I,NV_ID_ABNORMAL_STOPMIN_I_LEN,&val);
	  }
	  conf->setAbnormalStopMinI = (int32_t)val;
    INFO("AbnormalStop Min I = %d\n", conf->setAbnormalStopMinI);

    memset(array, 0, sizeof(array));
	  status = NvRead(NV_ID_DELAY_CH, NV_ID_DELAY_CH_LEN, (void*)array);
	  if(status != YSuccess || array[0] == 0xFFFFFFFF)
    {
      memset(array, 0, sizeof(array));
      array[RF_CH0] = RF_DEF_DELAY_CH0;
      array[RF_CH1] = RF_DEF_DELAY_CH1;
      array[RF_CH2] = RF_DEF_DELAY_CH2;
      array[RF_CH3] = RF_DEF_DELAY_CH3;
      array[RF_CH4] = RF_DEF_DELAY_CH4;
      array[RF_CH5] = RF_DEF_DELAY_CH5;
      array[RF_CH6] = RF_DEF_DELAY_CH6;
      NvWrite(NV_ID_DELAY_CH,NV_ID_DELAY_CH_LEN,array);
	  }
    
    conf->setDelay[RF_CH0] = array[RF_CH0];
    conf->setDelay[RF_CH1] = array[RF_CH1];
    conf->setDelay[RF_CH2] = array[RF_CH2];
    conf->setDelay[RF_CH3] = array[RF_CH3];
    conf->setDelay[RF_CH4] = array[RF_CH4];
    conf->setDelay[RF_CH5] = array[RF_CH5];
    conf->setDelay[RF_CH6] = array[RF_CH6];

    INFO("Delay CH0 = %d\n", conf->setDelay[RF_CH0]);
    INFO("Delay CH1 = %d\n", conf->setDelay[RF_CH1]);
    INFO("Delay CH2 = %d\n", conf->setDelay[RF_CH2]);
    INFO("Delay CH3 = %d\n", conf->setDelay[RF_CH3]);
    INFO("Delay CH4 = %d\n", conf->setDelay[RF_CH4]);
    INFO("Delay CH5 = %d\n", conf->setDelay[RF_CH5]);
    INFO("Delay CH6 = %d\n", conf->setDelay[RF_CH6]);
  }

}

#if 1
void nv_write_setFrequency(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: setFrequency = %d.\n", __func__, val);
  status = NvWrite(NV_ID_FREQUENCY,NV_ID_FREQUENCY_LEN,(void *)&val);
  if(status != YSuccess)
  {
	  INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setOutputVoltage(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_OUTPUTVOLTE,NV_ID_OUTPUTVOLTE_LEN,(void *)&val);
  if(status != YSuccess)
  {
	  INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}


void nv_write_setTBD(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_TBD,NV_ID_TBD_LEN,(void *)&val);
  if(status != YSuccess)
  {
	  INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setDuty(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_DUTY,NV_ID_DUTY_LEN,(void *)&val);
  if(status != YSuccess)
  {
	  INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setPRP(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_PRP,NV_ID_PRP_LEN,(void *)&val);
  if(status != YSuccess)
  {
	INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setSD(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_SD,NV_ID_SD_LEN,(void *)&val);
  if(status != YSuccess)
  {
	  INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setISI(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_ISI,NV_ID_ISI_LEN,(void *)&val);
  if(status != YSuccess)
  {
	INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setBI(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);

  status = NvWrite(NV_ID_BI,NV_ID_BI_LEN,(void *)&val);
  if(status != YSuccess)
  {
	INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setTD(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);

  status = NvWrite(NV_ID_TD,NV_ID_TD_LEN,(void *)&val);
  if(status != YSuccess)
  {
	INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}


void nv_write_setImpedance(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_IMPEDANCE,NV_ID_IMPEDANCE_LEN,(void *)&val);
  if(status != YSuccess)
  {
	INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setAbnormalStopMode(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;

  INFO("%s: val = %d.\n", __func__, val);
  status = NvWrite(NV_ID_ABNORMAL_STOPMODE,NV_ID_ABNORMAL_STOPMODE_LEN,(void *)&val);
  if(status != YSuccess)
  {
	INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}

void nv_write_setDelay(int32_t val)
{
  #ifdef USE_INTERNAL_FLASH
  int status;
  int32_t array[MAX_RF_CH];

  INFO("%s: val = %d.\n", __func__, val);

  memset(array, 0, sizeof(array));
  array[RF_CH0] = val;
  array[RF_CH1] = val;
  array[RF_CH2] = val;
  array[RF_CH3] = val;
  array[RF_CH4] = val;
  array[RF_CH5] = val;
  array[RF_CH6] = val;

  status = NvWrite(NV_ID_DELAY_CH,NV_ID_DELAY_CH_LEN,(void *)array);
  if(status != YSuccess)
  {
	  INFO("%s: NvWrite failed.\n", __func__);
  }
  #endif
}
#endif
