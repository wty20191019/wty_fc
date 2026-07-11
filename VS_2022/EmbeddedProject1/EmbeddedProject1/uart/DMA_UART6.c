#include "DMA_UART6.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx_conf.h"

#define UART6_RX_DMA_BUF_SIZE      (256U)
#define UART6_PRINTF_BUF_SIZE      (256U)

#define UART6_TX_GPIO_PORT         GPIOC
#define UART6_TX_GPIO_PIN          GPIO_Pin_6
#define UART6_TX_GPIO_SOURCE       GPIO_PinSource6

#define UART6_RX_GPIO_PORT         GPIOC
#define UART6_RX_GPIO_PIN          GPIO_Pin_7
#define UART6_RX_GPIO_SOURCE       GPIO_PinSource7

#define UART6_GPIO_AF              GPIO_AF_USART6

#define UART6_TX_STREAM            DMA2_Stream6
#define UART6_TX_CHANNEL           DMA_Channel_5
#define UART6_TX_TC_FLAG           DMA_FLAG_TCIF6
#define UART6_TX_FE_FLAG           DMA_FLAG_FEIF6
#define UART6_TX_DME_FLAG          DMA_FLAG_DMEIF6
#define UART6_TX_TE_FLAG           DMA_FLAG_TEIF6
#define UART6_TX_HT_FLAG           DMA_FLAG_HTIF6

#define UART6_RX_STREAM            DMA2_Stream1
#define UART6_RX_CHANNEL           DMA_Channel_5

static uint8_t g_uart6RxDmaBuf[UART6_RX_DMA_BUF_SIZE];
static uint16_t g_uart6RxReadIndex;

static void DMA_USART6_GPIO_Init(void)
{
    GPIO_InitTypeDef gpioInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_PinAFConfig(UART6_TX_GPIO_PORT, UART6_TX_GPIO_SOURCE, UART6_GPIO_AF);
    GPIO_PinAFConfig(UART6_RX_GPIO_PORT, UART6_RX_GPIO_SOURCE, UART6_GPIO_AF);

    gpioInit.GPIO_Pin = UART6_TX_GPIO_PIN | UART6_RX_GPIO_PIN;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpioInit);
}

static void DMA_USART6_Core_Init(uint32_t baudrate)
{
    USART_InitTypeDef usartInit;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    USART_DeInit(USART6);
    usartInit.USART_BaudRate = baudrate;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_No;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &usartInit);

    USART_DMACmd(USART6, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART6, ENABLE);
}

static void DMA_USART6_RX_Init(void)
{
    DMA_InitTypeDef dmaInit;

    DMA_DeInit(UART6_RX_STREAM);
    while (DMA_GetCmdStatus(UART6_RX_STREAM) != DISABLE)
    {
    }

    DMA_StructInit(&dmaInit);
    dmaInit.DMA_Channel = UART6_RX_CHANNEL;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    dmaInit.DMA_Memory0BaseAddr = (uint32_t)g_uart6RxDmaBuf;
    dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInit.DMA_BufferSize = UART6_RX_DMA_BUF_SIZE;
    dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInit.DMA_Mode = DMA_Mode_Circular;
    dmaInit.DMA_Priority = DMA_Priority_High;
    dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(UART6_RX_STREAM, &dmaInit);

    DMA_Cmd(UART6_RX_STREAM, ENABLE);
}

static void DMA_USART6_TX_Init(void)
{
    DMA_InitTypeDef dmaInit;

    DMA_DeInit(UART6_TX_STREAM);
    while (DMA_GetCmdStatus(UART6_TX_STREAM) != DISABLE)
    {
    }

    DMA_StructInit(&dmaInit);
    dmaInit.DMA_Channel = UART6_TX_CHANNEL;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    dmaInit.DMA_Memory0BaseAddr = (uint32_t)0;
    dmaInit.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dmaInit.DMA_BufferSize = 0;
    dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInit.DMA_Mode = DMA_Mode_Normal;
    dmaInit.DMA_Priority = DMA_Priority_High;
    dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(UART6_TX_STREAM, &dmaInit);
}

void DMA_USART6_Init(uint32_t baudrate)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_USART6_GPIO_Init();
    DMA_USART6_Core_Init(baudrate);
    DMA_USART6_RX_Init();
    DMA_USART6_TX_Init();

    g_uart6RxReadIndex = 0U;
}

void DMA_USART6_Send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    while (DMA_GetCmdStatus(UART6_TX_STREAM) != DISABLE)
    {
    }

    DMA_ClearFlag(UART6_TX_STREAM, UART6_TX_TC_FLAG | UART6_TX_FE_FLAG | UART6_TX_DME_FLAG | UART6_TX_TE_FLAG | UART6_TX_HT_FLAG);

    UART6_TX_STREAM->M0AR = (uint32_t)data;
    UART6_TX_STREAM->NDTR = len;
    DMA_Cmd(UART6_TX_STREAM, ENABLE);

    while (DMA_GetFlagStatus(UART6_TX_STREAM, UART6_TX_TC_FLAG) == RESET)
    {
    }

    DMA_Cmd(UART6_TX_STREAM, DISABLE);
    DMA_ClearFlag(UART6_TX_STREAM, UART6_TX_TC_FLAG | UART6_TX_FE_FLAG | UART6_TX_DME_FLAG | UART6_TX_TE_FLAG | UART6_TX_HT_FLAG);
}

void DMA_USART6_SendString(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    DMA_USART6_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

void Serial6_Printf(char *format, ...)
{
    char printBuf[UART6_PRINTF_BUF_SIZE];
    int len;
    va_list args;

    if (format == NULL)
    {
        return;
    }

    va_start(args, format);
    len = vsnprintf(printBuf, sizeof(printBuf), format, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if ((uint32_t)len >= UART6_PRINTF_BUF_SIZE)
    {
        len = (int)(UART6_PRINTF_BUF_SIZE - 1U);
    }

    DMA_USART6_Send((const uint8_t *)printBuf, (uint16_t)len);
}

uint16_t DMA_USART6_Read(uint8_t *out, uint16_t maxLen)
{
    uint16_t writeIndex;
    uint16_t available;
    uint16_t toCopy;

    if ((out == NULL) || (maxLen == 0U))
    {
        return 0U;
    }

    writeIndex = (uint16_t)(UART6_RX_DMA_BUF_SIZE - UART6_RX_STREAM->NDTR);

    if (writeIndex >= g_uart6RxReadIndex)
    {
        available = (uint16_t)(writeIndex - g_uart6RxReadIndex);
    }
    else
    {
        available = (uint16_t)(UART6_RX_DMA_BUF_SIZE - g_uart6RxReadIndex + writeIndex);
    }

    toCopy = (available > maxLen) ? maxLen : available;
    for (uint16_t i = 0U; i < toCopy; ++i)
    {
        out[i] = g_uart6RxDmaBuf[g_uart6RxReadIndex];
        g_uart6RxReadIndex++;
        if (g_uart6RxReadIndex >= UART6_RX_DMA_BUF_SIZE)
        {
            g_uart6RxReadIndex = 0U;
        }
    }

    return toCopy;
}
