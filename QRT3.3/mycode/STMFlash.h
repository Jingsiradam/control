#ifndef __STMFLASH_H
#define __STMFLASH_H
#include "stm32f4xx_hal.h"
#include "pid.h"

//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
#define FLASH_WAITETIME  50000          //FLASH�ȴ���ʱʱ��

#define FIRMWARE_SIZE (96*1024)            // ����洢����80K���ң�Ԥ��128K

#define FIRMWARE_START_ADDR (FLASH_BASE)	
// ���ݱ����� ADDR_FLASH_SECTOR_3 �����СԼΪ74KB
#define CONFIG_PARAM_ADDR 	(FLASH_BASE + FIRMWARE_SIZE)	

//STM32F405 FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) //����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) //����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) //����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) //����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) //����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) //����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) //����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) //����7��ʼ��ַ, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) //����7��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) //����7��ʼ��ַ, 128 Kbytes 
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) //����7��ʼ��ַ, 128 Kbytes
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) //����7��ʼ��ַ, 128 Kbytes

uint32_t STMFLASH_ReadWord(uint32_t faddr);		  	//������  
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����



#endif
