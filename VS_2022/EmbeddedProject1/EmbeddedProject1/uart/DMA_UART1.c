#include "DMA_UART1.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define UART1_RX_DMA_BUF_SIZE      (256U)
#define UART1_PRINTF_BUF_SIZE      (256U)

#define UART1_TX_GPIO_PORT         GPIOA
#define UART1_TX_GPIO_PIN          GPIO_Pin_9
#define UART1_TX_GPIO_SOURCE       GPIO_PinSource9

#define UART1_RX_GPIO_PORT         GPIOA
#define UART1_RX_GPIO_PIN          GPIO_Pin_10
#define UART1_RX_GPIO_SOURCE       GPIO_PinSource10

#define UART1_GPIO_AF              GPIO_AF_USART1

#define UART1_DMA                  DMA2
#define UART1_TX_STREAM            DMA2_Stream7
#define UART1_TX_CHANNEL           DMA_Channel_4
#define UART1_TX_TC_FLAG           DMA_FLAG_TCIF7
#define UART1_TX_FE_FLAG           DMA_FLAG_FEIF7
#define UART1_TX_DME_FLAG          DMA_FLAG_DMEIF7
#define UART1_TX_TE_FLAG           DMA_FLAG_TEIF7
#define UART1_TX_HT_FLAG           DMA_FLAG_HTIF7

#define UART1_RX_STREAM            DMA2_Stream2
#define UART1_RX_CHANNEL           DMA_Channel_4

static uint8_t g_uart1TxSingleByte;
static uint8_t g_uart1RxDmaBuf[UART1_RX_DMA_BUF_SIZE];
static uint16_t g_uart1RxReadIndex;

static void DMA_USART1_GPIO_Init(void)//初始化USART1的GPIO引脚
{
    GPIO_InitTypeDef gpioInit;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(UART1_TX_GPIO_PORT, UART1_TX_GPIO_SOURCE, UART1_GPIO_AF);
    GPIO_PinAFConfig(UART1_RX_GPIO_PORT, UART1_RX_GPIO_SOURCE, UART1_GPIO_AF);

    gpioInit.GPIO_Pin = UART1_TX_GPIO_PIN | UART1_RX_GPIO_PIN;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpioInit);
}

static void DMA_USART1_Core_Init(uint32_t baudrate)//初始化USART1的核心功能（波特率、DMA使能等）
{
    USART_InitTypeDef usartInit;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_DeInit(USART1);
    usartInit.USART_BaudRate = baudrate;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_No;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usartInit);

    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

static void DMA_USART1_RX_Init(void)//初始化USART1的DMA接收功能
{
    DMA_InitTypeDef dmaInit;

    DMA_DeInit(UART1_RX_STREAM);
    while (DMA_GetCmdStatus(UART1_RX_STREAM) != DISABLE)
    {
    }

    DMA_StructInit(&dmaInit);
    dmaInit.DMA_Channel = UART1_RX_CHANNEL;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    dmaInit.DMA_Memory0BaseAddr = (uint32_t)g_uart1RxDmaBuf;
    dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInit.DMA_BufferSize = UART1_RX_DMA_BUF_SIZE;
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
    DMA_Init(UART1_RX_STREAM, &dmaInit);

    DMA_Cmd(UART1_RX_STREAM, ENABLE);
}

static void DMA_USART1_TX_Init(void)//初始化USART1的DMA发送功能
{
    DMA_InitTypeDef dmaInit;

    DMA_DeInit(UART1_TX_STREAM);
    while (DMA_GetCmdStatus(UART1_TX_STREAM) != DISABLE)
    {
    }

    DMA_StructInit(&dmaInit);
    dmaInit.DMA_Channel = UART1_TX_CHANNEL;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
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
    DMA_Init(UART1_TX_STREAM, &dmaInit);
}

void DMA_USART1_Init(uint32_t baudrate)//初始化USART1的DMA功能
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_USART1_GPIO_Init();
    DMA_USART1_Core_Init(baudrate);
    DMA_USART1_RX_Init();
    DMA_USART1_TX_Init();

    g_uart1RxReadIndex = 0U;
}

void DMA_USART1_Send(const uint8_t *data, uint16_t len)//通过DMA发送数据
{
    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    while (DMA_GetCmdStatus(UART1_TX_STREAM) != DISABLE)
    {
    }

    DMA_ClearFlag(UART1_TX_STREAM, UART1_TX_TC_FLAG | UART1_TX_FE_FLAG | UART1_TX_DME_FLAG | UART1_TX_TE_FLAG | UART1_TX_HT_FLAG);

    UART1_TX_STREAM->M0AR = (uint32_t)data;
    UART1_TX_STREAM->NDTR = len;
    DMA_Cmd(UART1_TX_STREAM, ENABLE);

    while (DMA_GetFlagStatus(UART1_TX_STREAM, UART1_TX_TC_FLAG) == RESET)
    {
    }

    DMA_Cmd(UART1_TX_STREAM, DISABLE);
    DMA_ClearFlag(UART1_TX_STREAM, UART1_TX_TC_FLAG | UART1_TX_FE_FLAG | UART1_TX_DME_FLAG | UART1_TX_TE_FLAG | UART1_TX_HT_FLAG);
}

void DMA_USART1_SendString(const char *str)//通过DMA发送字符串
{
    if (str == NULL)
    {
        return;
    }

    DMA_USART1_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

void Serial1_Printf(char *format, ...) //串口1格式化输出
{
    char printBuf[UART1_PRINTF_BUF_SIZE];
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

    if ((uint32_t)len >= UART1_PRINTF_BUF_SIZE)
    {
        len = (int)(UART1_PRINTF_BUF_SIZE - 1U);
    }

    DMA_USART1_Send((const uint8_t *)printBuf, (uint16_t)len);
}

uint16_t DMA_USART1_Read(uint8_t *out, uint16_t maxLen)//从DMA接收缓冲区读取数据
{
    uint16_t writeIndex;
    uint16_t available;
    uint16_t toCopy;

    if ((out == NULL) || (maxLen == 0U))
    {
        return 0U;
    }

    writeIndex = (uint16_t)(UART1_RX_DMA_BUF_SIZE - UART1_RX_STREAM->NDTR);

    if (writeIndex >= g_uart1RxReadIndex)
    {
        available = (uint16_t)(writeIndex - g_uart1RxReadIndex);
    }
    else
    {
        available = (uint16_t)(UART1_RX_DMA_BUF_SIZE - g_uart1RxReadIndex + writeIndex);
    }

    toCopy = (available > maxLen) ? maxLen : available;
    for (uint16_t i = 0U; i < toCopy; ++i)
    {
        out[i] = g_uart1RxDmaBuf[g_uart1RxReadIndex];
        g_uart1RxReadIndex++;
        if (g_uart1RxReadIndex >= UART1_RX_DMA_BUF_SIZE)
        {
            g_uart1RxReadIndex = 0U;
        }
    }

    return toCopy;
}

int fputc(int ch, FILE *f)//重定向printf到USART1
{
    (void)f;

    g_uart1TxSingleByte = (uint8_t)ch;
    DMA_USART1_Send(&g_uart1TxSingleByte, 1U);

    return ch;
}



