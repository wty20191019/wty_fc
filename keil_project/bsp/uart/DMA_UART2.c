#include "DMA_UART2.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define UART2_RX_DMA_BUF_SIZE      (256U)
#define UART2_PRINTF_BUF_SIZE      (256U)

#define UART2_TX_GPIO_PORT         GPIOA
#define UART2_TX_GPIO_PIN          GPIO_Pin_2
#define UART2_TX_GPIO_SOURCE       GPIO_PinSource2

#define UART2_RX_GPIO_PORT         GPIOA
#define UART2_RX_GPIO_PIN          GPIO_Pin_3
#define UART2_RX_GPIO_SOURCE       GPIO_PinSource3

#define UART2_GPIO_AF              GPIO_AF_USART2

#define UART2_TX_STREAM            DMA1_Stream6
#define UART2_TX_CHANNEL           DMA_Channel_4
#define UART2_TX_TC_FLAG           DMA_FLAG_TCIF6
#define UART2_TX_FE_FLAG           DMA_FLAG_FEIF6
#define UART2_TX_DME_FLAG          DMA_FLAG_DMEIF6
#define UART2_TX_TE_FLAG           DMA_FLAG_TEIF6
#define UART2_TX_HT_FLAG           DMA_FLAG_HTIF6

#define UART2_RX_STREAM            DMA1_Stream5
#define UART2_RX_CHANNEL           DMA_Channel_4

static uint8_t g_uart2RxDmaBuf[UART2_RX_DMA_BUF_SIZE];
static uint16_t g_uart2RxReadIndex;

static void DMA_USART2_GPIO_Init(void)
{
    GPIO_InitTypeDef gpioInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(UART2_TX_GPIO_PORT, UART2_TX_GPIO_SOURCE, UART2_GPIO_AF);
    GPIO_PinAFConfig(UART2_RX_GPIO_PORT, UART2_RX_GPIO_SOURCE, UART2_GPIO_AF);

    gpioInit.GPIO_Pin = UART2_TX_GPIO_PIN | UART2_RX_GPIO_PIN;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpioInit);
}

static void DMA_USART2_Core_Init(uint32_t baudrate)
{
    USART_InitTypeDef usartInit;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    USART_DeInit(USART2);
    usartInit.USART_BaudRate = baudrate;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_No;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usartInit);

    USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

static void DMA_USART2_RX_Init(void)
{
    DMA_InitTypeDef dmaInit;

    DMA_DeInit(UART2_RX_STREAM);
    while (DMA_GetCmdStatus(UART2_RX_STREAM) != DISABLE)
    {
    }

    DMA_StructInit(&dmaInit);
    dmaInit.DMA_Channel = UART2_RX_CHANNEL;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    dmaInit.DMA_Memory0BaseAddr = (uint32_t)g_uart2RxDmaBuf;
    dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInit.DMA_BufferSize = UART2_RX_DMA_BUF_SIZE;
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
    DMA_Init(UART2_RX_STREAM, &dmaInit);

    DMA_Cmd(UART2_RX_STREAM, ENABLE);
}

static void DMA_USART2_TX_Init(void)
{
    DMA_InitTypeDef dmaInit;

    DMA_DeInit(UART2_TX_STREAM);
    while (DMA_GetCmdStatus(UART2_TX_STREAM) != DISABLE)
    {
    }

    DMA_StructInit(&dmaInit);
    dmaInit.DMA_Channel = UART2_TX_CHANNEL;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
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
    DMA_Init(UART2_TX_STREAM, &dmaInit);
}

void DMA_USART2_Init(uint32_t baudrate)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    DMA_USART2_GPIO_Init();
    DMA_USART2_Core_Init(baudrate);
    DMA_USART2_RX_Init();
    DMA_USART2_TX_Init();

    g_uart2RxReadIndex = 0U;
}

void DMA_USART2_Send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    while (DMA_GetCmdStatus(UART2_TX_STREAM) != DISABLE)
    {
    }

    DMA_ClearFlag(UART2_TX_STREAM, UART2_TX_TC_FLAG | UART2_TX_FE_FLAG | UART2_TX_DME_FLAG | UART2_TX_TE_FLAG | UART2_TX_HT_FLAG);

    UART2_TX_STREAM->M0AR = (uint32_t)data;
    UART2_TX_STREAM->NDTR = len;
    DMA_Cmd(UART2_TX_STREAM, ENABLE);

    while (DMA_GetFlagStatus(UART2_TX_STREAM, UART2_TX_TC_FLAG) == RESET)
    {
    }

    DMA_Cmd(UART2_TX_STREAM, DISABLE);
    DMA_ClearFlag(UART2_TX_STREAM, UART2_TX_TC_FLAG | UART2_TX_FE_FLAG | UART2_TX_DME_FLAG | UART2_TX_TE_FLAG | UART2_TX_HT_FLAG);
}

void DMA_USART2_SendString(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    DMA_USART2_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

void Serial2_Printf(char *format, ...)
{
    char printBuf[UART2_PRINTF_BUF_SIZE];
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

    if ((uint32_t)len >= UART2_PRINTF_BUF_SIZE)
    {
        len = (int)(UART2_PRINTF_BUF_SIZE - 1U);
    }

    DMA_USART2_Send((const uint8_t *)printBuf, (uint16_t)len);
}

uint16_t DMA_USART2_Read(uint8_t *out, uint16_t maxLen)
{
    uint16_t writeIndex;
    uint16_t available;
    uint16_t toCopy;

    if ((out == NULL) || (maxLen == 0U))
    {
        return 0U;
    }

    writeIndex = (uint16_t)(UART2_RX_DMA_BUF_SIZE - UART2_RX_STREAM->NDTR);

    if (writeIndex >= g_uart2RxReadIndex)
    {
        available = (uint16_t)(writeIndex - g_uart2RxReadIndex);
    }
    else
    {
        available = (uint16_t)(UART2_RX_DMA_BUF_SIZE - g_uart2RxReadIndex + writeIndex);
    }

    toCopy = (available > maxLen) ? maxLen : available;
    for (uint16_t i = 0U; i < toCopy; ++i)
    {
        out[i] = g_uart2RxDmaBuf[g_uart2RxReadIndex];
        g_uart2RxReadIndex++;
        if (g_uart2RxReadIndex >= UART2_RX_DMA_BUF_SIZE)
        {
            g_uart2RxReadIndex = 0U;
        }
    }

    return toCopy;
}
