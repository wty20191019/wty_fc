#ifndef PID_FLASH_STORE_H
#define PID_FLASH_STORE_H

#include <stdint.h>

#include "all_control.h"
#include "stm32f4xx_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default location: STM32F405RG 1MB flash, sector 11 (0x080E0000..0x080FFFFF)
// If your linker script uses the last sector, override these macros in the build.
#ifndef PID_FLASH_STORE_BASE_ADDR
#define PID_FLASH_STORE_BASE_ADDR ((uint32_t)0x080E0000U)
#endif

#ifndef PID_FLASH_STORE_SECTOR
#define PID_FLASH_STORE_SECTOR FLASH_Sector_11
#endif

#define PID_FLASH_STORE_VERSION (1U)

typedef struct
{
    float kp;
    float ki;
    float kd;
} PID_FlashGains_t;

uint8_t PID_FlashStore_Load(PID_FlashGains_t gains[ALL_PID_COUNT]);
uint8_t PID_FlashStore_Save(const PID_FlashGains_t gains[ALL_PID_COUNT]);

#ifdef __cplusplus
}
#endif

#endif // PID_FLASH_STORE_H
