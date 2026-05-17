#include "pid_flash_store.h"

#include "stm32f4xx.h"
#include "stm32f4xx_flash.h"

#include <stddef.h>
#include <string.h>

#define PID_FLASH_STORE_MAGIC (0x53444950U) // 'PIDS' little-endian

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t lengthBytes;
    PID_FlashGains_t gains[ALL_PID_COUNT];
    uint32_t crc32;
} PID_FlashRecord_t;

static uint32_t PID_FlashStore_Crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;

    while (len-- != 0U)
    {
        crc ^= (uint32_t)(*p++);
        for (uint32_t i = 0U; i < 8U; ++i)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }

    return ~crc;
}

static uint8_t PID_FlashStore_RecordValid(const PID_FlashRecord_t *rec)
{
    if (rec == NULL)
    {
        return 0U;
    }

    if (rec->magic != PID_FLASH_STORE_MAGIC)
    {
        return 0U;
    }

    if (rec->version != (uint16_t)PID_FLASH_STORE_VERSION)
    {
        return 0U;
    }

    if (rec->lengthBytes != (uint16_t)sizeof(PID_FlashRecord_t))
    {
        return 0U;
    }

    uint32_t calc = PID_FlashStore_Crc32(rec, offsetof(PID_FlashRecord_t, crc32));
    return (calc == rec->crc32) ? 1U : 0U;
}

uint8_t PID_FlashStore_Load(PID_FlashGains_t gains[ALL_PID_COUNT])
{
    if (gains == NULL)
    {
        return 0U;
    }

    const PID_FlashRecord_t *rec = (const PID_FlashRecord_t *)PID_FLASH_STORE_BASE_ADDR;

    if (PID_FlashStore_RecordValid(rec) == 0U)
    {
        return 0U;
    }

    memcpy(gains, rec->gains, sizeof(rec->gains));
    return 1U;
}

uint8_t PID_FlashStore_Save(const PID_FlashGains_t gains[ALL_PID_COUNT])
{
    if (gains == NULL)
    {
        return 0U;
    }

    PID_FlashRecord_t rec;
    memset(&rec, 0, sizeof(rec));

    rec.magic = PID_FLASH_STORE_MAGIC;
    rec.version = (uint16_t)PID_FLASH_STORE_VERSION;
    rec.lengthBytes = (uint16_t)sizeof(PID_FlashRecord_t);
    memcpy(rec.gains, gains, sizeof(rec.gains));
    rec.crc32 = PID_FlashStore_Crc32(&rec, offsetof(PID_FlashRecord_t, crc32));

    if ((sizeof(PID_FlashRecord_t) % 4U) != 0U)
    {
        return 0U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_Status st = FLASH_EraseSector(PID_FLASH_STORE_SECTOR, VoltageRange_3);
    if (st != FLASH_COMPLETE)
    {
        FLASH_Lock();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    const uint32_t *src = (const uint32_t *)&rec;
    uint32_t addr = PID_FLASH_STORE_BASE_ADDR;
    const uint32_t wordCount = (uint32_t)(sizeof(PID_FlashRecord_t) / 4U);

    for (uint32_t i = 0U; i < wordCount; ++i)
    {
        st = FLASH_ProgramWord(addr, src[i]);
        if (st != FLASH_COMPLETE)
        {
            FLASH_Lock();
            if (primask == 0U)
            {
                __enable_irq();
            }
            return 0U;
        }
        addr += 4U;
    }

    FLASH_Lock();

    if (primask == 0U)
    {
        __enable_irq();
    }

    // verify
    PID_FlashRecord_t verify;
    memcpy(&verify, (const void *)PID_FLASH_STORE_BASE_ADDR, sizeof(verify));
    return (PID_FlashStore_RecordValid(&verify) != 0U) ? 1U : 0U;
}
