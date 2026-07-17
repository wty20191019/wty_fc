#include "DMA_UART1.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

/* 环形缓冲区大小（可根据需要调整）*/
#define RX_BUF_SIZE     256

/* 环形缓冲区数据结构 */
static uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_write = 0; // 中断写入位置
static volatile uint16_t rx_read  = 0; // 用户读取位置

/**
 * @brief  初始化USART1（不使用DMA，使用接收中断）
 * @param  baudrate : 波特率
 */
void DMA_USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    /* 1. 时钟使能 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* 2. GPIO配置：PA9(TX)复用推挽输出，PA10(RX)浮空输入 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 3. USART1配置 */
    USART_DeInit(USART1);
    usart.USART_BaudRate            = baudrate;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);

    /* 4. 使能接收中断（每收到一个字节触发）*/
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 5. 配置NVIC */
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* 6. 使能USART1 */
    USART_Cmd(USART1, ENABLE);

    /* 初始化环形缓冲区指针 */
    rx_write = 0;
    rx_read  = 0;
}

/**
 * @brief  通过USART1发送数据（轮询阻塞）
 * @param  data : 待发送数据指针
 * @param  len  : 数据长度
 */
void DMA_USART1_Send(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        /* 等待发送数据寄存器空 */
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) ;
        USART_SendData(USART1, data[i]);
    }
    /* 等待最后一个字节发送完成（TC标志）*/
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) ;
}

/**
 * @brief  通过USART1发送字符串
 * @param  str : 以'\0'结尾的字符串
 */
void DMA_USART1_SendString(const char *str)
{
    uint16_t len = 0;
    const char *p = str;
    while (*p++) len++;
    DMA_USART1_Send((const uint8_t *)str, len);
}

/**
 * @brief  从接收环形缓冲区读取数据
 * @param  out    : 存放数据的用户缓冲区
 * @param  maxLen : 最大读取字节数
 * @return 实际读取的字节数
 */
uint16_t DMA_USART1_Read(uint8_t *out, uint16_t maxLen)
{
    uint16_t cnt = 0;
    uint16_t available;

    /* 关中断保护（若在多线程环境下需要，这里简单处理）*/
    __disable_irq();
    if (rx_write >= rx_read)
        available = rx_write - rx_read;
    else
        available = RX_BUF_SIZE - rx_read + rx_write;
    __enable_irq();

    if (available == 0) return 0;

    if (available > maxLen)
        available = maxLen;

    while (cnt < available)
    {
        out[cnt++] = rx_buf[rx_read++];
        if (rx_read >= RX_BUF_SIZE)
            rx_read = 0;
    }
    return cnt;
}

/* ================== 中断服务函数 ================== */

/**
 * @brief  USART1接收中断服务函数
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART1);

        /* 将数据写入环形缓冲区 */
        rx_buf[rx_write++] = data;
        if (rx_write >= RX_BUF_SIZE)
            rx_write = 0;

        /* 如果缓冲区满，可以丢弃旧数据（此处简单覆盖，不处理溢出）*/
        /* 实际应用中可增加溢出计数或丢弃最旧数据 */
    }
}