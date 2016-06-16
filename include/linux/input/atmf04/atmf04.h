#ifndef __ATMF04_EFLASH_H__
#define __ATMF04_EFLASH_H__

/****************************************************************************
* Project Setting (Model)
****************************************************************************/


#define SZ_PAGE_DATA                64
#define FW_DATA_PAGE               	115

#define ADDR_EFLA_STS               0xFF	//eflash status register
#define ADDR_EFLA_PAGE_L            0xFD	//eflash page
#define ADDR_EFLA_PAGE_H            0xFE	//eflash page
#define ADDR_EFLA_CTRL              0xFC	//eflash control register

#define CMD_EFL_L_WR                0x01	//Eflash Write
#define CMD_EFL_RD                  0x03	//Eflash Read
#define CMD_EFL_ERASE_ALL           0x07	//Eflash All Page Erase

#define CMD_EUM_WR                  0x21	//Extra user memory write
#define CMD_EUM_RD                  0x23	//Extra user memory read
#define CMD_EUM_ERASE               0x25	//Extra user memory erase

#define FLAG_DONE                   0x03
#define FLAG_DONE_ERASE             0x02

#define FLAG_FUSE                   1
#define FLAG_FW                     2

#define FL_EFLA_TIMEOUT_CNT         200

#define RTN_FAIL                    0
#define RTN_SUCC                    1
#define RTN_TIMEOUT                 2

#define ON                          1
#define OFF                         2

// touch key notifier
#define ATMF04_LCD_EVENT_ON			0
#define ATMF04_LCD_EVENT_OFF		1

int touch_key_register_client(struct notifier_block *nb);
int touch_key_unregister_client(struct notifier_block *nb);
int touch_key_notifier_call_chain(unsigned long val, void *v);

#endif


