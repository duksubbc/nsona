#ifndef _HAL_NV_H
#define	_HAL_NV_H

#define NV_SUCCESS          0
#define NV_NO_LOG           (uint16_t)0xFFFF
#define NV_INVALID_OFFSET   (uint16_t)0xFFFE

#define EPERM        1  /* Operation not permitted */
#define ENOENT       2  /* No such file or directory */
#define ESRCH        3  /* No such process */
#define EINTR        4  /* Interrupted system call */
#define EIO          5  /* I/O error */
#define ENXIO        6  /* No such device or address */
#define E2BIG        7  /* Argument list too long */
#define ENOEXEC      8  /* Exec format error */
#define EBADF        9  /* Bad file number */
#define ECHILD      10  /* No child processes */
#define EAGAIN      11  /* Try again */
#define ENOMEM      12  /* Out of memory */
#define EACCES      13  /* Permission denied */
#define EFAULT      14  /* Bad address */
#define ENOTBLK     15  /* Block device required */
#define EBUSY       16  /* Device or resource busy */
#define EEXIST      17  /* File exists */
#define EXDEV       18  /* Cross-device link */
#define ENODEV      19  /* No such device */
#define ENOTDIR     20  /* Not a directory */
#define EISDIR      21  /* Is a directory */
#define EINVAL      22  /* Invalid argument */
#define ENFILE      23  /* File table overflow */
#define EMFILE      24  /* Too many open files */
#define ENOTTY      25  /* Not a typewriter */
#define ETXTBSY     26  /* Text file busy */
#define EFBIG       27  /* File too large */
#define ENOSPC      28  /* No space left on device */
#define ESPIPE      29  /* Illegal seek */
#define EROFS       30  /* Read-only file system */
#define EMLINK      31  /* Too many links */
#define EPIPE       32  /* Broken pipe */
#define EDOM        33  /* Math argument out of domain of func */
#define ERANGE      34  /* Math result not representable */

typedef enum{
  YSuccess,
  YEInvalidCommand,
  YEInvalidState,
  YEInvalidParameter,
  YEStartWhenNotWear,
  YEMemUnavailable,
  YEConnectionFailure,
  YEInvalidSubsystem,
  YEUnsupportedOperation,
  YTestFailure,
} YStatus_t;


/*
warnnig!!! you must add new item to last one (before NV_ID_MAX_NUM)
*/\
enum {
NV_FLASH_INIT_FLAG = 0,
NV_ID_FREQUENCY,
NV_ID_OUTPUTVOLTE,
NV_ID_TBD,
NV_ID_DUTY,
NV_ID_PRP,
NV_ID_SD,
NV_ID_ISI,
NV_ID_BI,
NV_ID_TD,
NV_ID_IMPEDANCE,
NV_ID_ABNORMAL_STOPMODE,
NV_ID_ABNORMAL_STOPMAX_V,
NV_ID_ABNORMAL_STOPMIN_V,
NV_ID_ABNORMAL_STOPMAX_I,
NV_ID_ABNORMAL_STOPMIN_I,
NV_ID_DELAY_CH,
NV_ID_MAX_NUM,
NV_ID_INVALID = 0xFFFF,
};


#define NV_FLASH_INIT_FLAG_LEN		4
#define	NV_ID_FREQUENCY_LEN		4
#define NV_ID_OUTPUTVOLTE_LEN 		4
#define NV_ID_TBD_LEN			4
#define NV_ID_DUTY_LEN			4
#define NV_ID_PRP_LEN			4
#define NV_ID_SD_LEN			4
#define NV_ID_ISI_LEN			4
#define NV_ID_BI_LEN			4
#define NV_ID_TD_LEN			4
#define NV_ID_IMPEDANCE_LEN		4
#define NV_ID_ABNORMAL_STOPMODE_LEN	4

#define NV_ID_ABNORMAL_STOPMAX_V_LEN	4
#define NV_ID_ABNORMAL_STOPMIN_V_LEN	4
#define NV_ID_ABNORMAL_STOPMAX_I_LEN	4
#define NV_ID_ABNORMAL_STOPMIN_I_LEN	4

#define NV_ID_DELAY_CH_LEN			      (4*MAX_RF_CH)

#define	NV_ID_SERIAL_NUMBER_LEN		28

#define NVID_HDR_LEN        sizeof(NvIdHdr_t)
/*********************************************************************
* MACROS
*/

/*********************************************************************
* TYPEDEFS
*/

typedef struct nv_id_table {
  uint16_t id;
  uint16_t lens;
} NvIdHdr_t;
/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* FUNCTIONS
*/

/* nv for user memory
 */

// Initialize NV service
extern void NvInit( void *p );
// Initialize an item in NV
extern int  NvItemInit( uint16_t id, uint16_t len, void *buf );

extern int  NvWriteDefault(void);

extern int  NvCheckAndReset(void);
extern void NvReadOrInit(uint16_t nvID, uint16_t len, void * pDstVal, const void* pInitVal);

// Read an NV attribute
extern int NvRead( uint16_t id, uint16_t len, void *buf );
// Write an NV attribute
extern int NvWrite( uint16_t id, uint16_t len, void *buf );

// Get the length of an NV item.
extern uint16_t NvItemLen( uint16_t id );
#if defined(CONFIG_SERIAL)
// Delete an NV item.
extern int NvDelete( uint16_t id );
extern int NvSerialNoWrite(void *buf, uint16_t len);
extern int NvSerialNoRead(uint8_t *buf);
#endif

extern void NvReadAll(void *env);

#if 0
extern void nv_write_setFrequency(void);
extern void nv_write_setOutputVoltage(void);

extern void nv_write_setDuty(void);
extern void nv_write_setPRP(void);
extern void nv_write_setSD(void);

extern void nv_write_setISI(void);
extern void nv_write_setBI(void);

extern void nv_write_setTD(void);
extern void nv_write_setImpedance(void);
extern void nv_write_setAbnormalStopMode(void);

extern void nv_write_setDelay(void);
#endif

#endif
