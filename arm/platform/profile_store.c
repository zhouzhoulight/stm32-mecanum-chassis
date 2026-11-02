#include "stm32f4xx_hal.h"
#include "profile_store.h"
#include <string.h>
#include <stdint.h>
#ifndef PROFILE_FLASH_BASE
#define PROFILE_FLASH_BASE ((uintptr_t)0x08060000U)
#endif
#define RECORD_BYTES 256U
#define RECORDS 512U
#define RECORD_MAGIC 0x394D5241U
static uint32_t checksum(const unsigned char *p,uint32_t n)
{
    uint32_t value=2166136261U;while(n--){value^=*p++;value*=16777619U;}return value;
}
static const unsigned char *record(uint32_t i){return (const unsigned char*)(PROFILE_FLASH_BASE+i*RECORD_BYTES);}
static int valid(uint32_t i,uint32_t bytes,uint32_t *seq)
{
    const uint32_t *h=(const uint32_t*)record(i);
    if(h[0]!=RECORD_MAGIC||h[2]!=bytes||bytes>240U||h[3]!=checksum(record(i)+16U,bytes))return 0;
    *seq=h[1];return 1;
}
int Board_CalRead(void *payload,uint32_t bytes)
{
    uint32_t i,seq,best=0,index=RECORDS;
    if(!payload||!bytes||bytes>240U)return 0;
    for(i=0;i<RECORDS;i++)if(valid(i,bytes,&seq)&&(index==RECORDS||seq>best)){best=seq;index=i;}
    if(index==RECORDS)return 0;memcpy(payload,record(index)+16U,bytes);return 1;
}
static int word(uintptr_t address,uint32_t value)
{
    return value==0xFFFFFFFFU||HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)address,value)==HAL_OK;
}
int Board_CalWrite(const void *payload,uint32_t bytes)
{
    uint32_t i,j,seq,best=0,index=RECORDS,error,header[4],value;uintptr_t address;int ok=1;
    if(!payload||!bytes||bytes>240U)return 0;
    for(i=0;i<RECORDS;i++){
        int erased=1;const uint32_t *p=(const uint32_t*)record(i);
        if(valid(i,bytes,&seq)&&seq>best)best=seq;
        for(j=0;j<RECORD_BYTES/4U;j++)if(p[j]!=0xFFFFFFFFU){erased=0;break;}
        if(erased&&index==RECORDS)index=i;
    }
    if(HAL_FLASH_Unlock()!=HAL_OK)return 0;
#ifndef PROFILE_STORE_TEST
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
#endif
    if(index==RECORDS){
        FLASH_EraseInitTypeDef erase={0};erase.TypeErase=FLASH_TYPEERASE_SECTORS;
        erase.Sector=FLASH_SECTOR_7;erase.NbSectors=1;erase.VoltageRange=FLASH_VOLTAGE_RANGE_3;
        if(HAL_FLASHEx_Erase(&erase,&error)!=HAL_OK)ok=0;index=0;
    }
    address=PROFILE_FLASH_BASE+index*RECORD_BYTES;
    header[0]=RECORD_MAGIC;header[1]=best+1U;header[2]=bytes;header[3]=checksum(payload,bytes);
    /* Payload and metadata first; final magic word commits a complete record. */
    for(i=0;ok&&i<bytes;i+=4){value=0xFFFFFFFFU;j=bytes-i<4U?bytes-i:4U;memcpy(&value,(const unsigned char*)payload+i,j);ok=word(address+16U+i,value);}
    for(i=1;ok&&i<4;i++)ok=word(address+i*4U,header[i]);
    if(ok)ok=word(address,header[0]);
    HAL_FLASH_Lock();FLASH_FlushCaches();if(!ok||!valid(index,bytes,&seq))return 0;
    return !memcmp(payload,record(index)+16U,bytes);
}
