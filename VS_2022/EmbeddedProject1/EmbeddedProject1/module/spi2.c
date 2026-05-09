#include "spi2.h"

#define SPI2_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define SPI2_GPIO_PORT         GPIOB
#define SPI2_CS_PIN            GPIO_Pin_12
#define SPI2_SCK_PIN           GPIO_Pin_13
#define SPI2_MISO_PIN          GPIO_Pin_14
#define SPI2_MOSI_PIN          GPIO_Pin_15

#define SPI2_PERIPH_CLK        RCC_APB1Periph_SPI2
#define SPI2_TIMEOUT_DEFAULT   20000U

static uint32_t spi2_timeout_value(uint32_t timeout)
{
    return (timeout == 0U) ? SPI2_TIMEOUT_DEFAULT : timeout;
}

void spi2_init(uint16_t prescaler)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(SPI2_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(SPI2_PERIPH_CLK, ENABLE);

    /* Configure SCK/MISO/MOSI as AF */
    GPIO_PinAFConfig(SPI2_GPIO_PORT, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_GPIO_PORT, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_GPIO_PORT, GPIO_PinSource15, GPIO_AF_SPI2);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN | SPI2_MISO_PIN | SPI2_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);

    /* Configure CS as GPIO output, set high (inactive) */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SPI2_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);
    GPIO_SetBits(SPI2_GPIO_PORT, SPI2_CS_PIN);

    /* SPI peripheral init */
    SPI_I2S_DeInit(SPI2);
    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = prescaler;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPI_Cmd(SPI2, ENABLE);
}

SPI2_Status spi2_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout)
{
    if ((tx == 0) && (rx == 0))
    {
        return SPI2_ERROR;
    }

    timeout = spi2_timeout_value(timeout);

    /* Assert CS (active low) */
    GPIO_ResetBits(SPI2_GPIO_PORT, SPI2_CS_PIN);

    while (len--)
    {
        uint32_t t = timeout;
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
        {
            if (t-- == 0U)
            {
                GPIO_SetBits(SPI2_GPIO_PORT, SPI2_CS_PIN);
                return SPI2_TIMEOUT;
            }
        }

        uint8_t out = (tx != 0) ? *tx++ : 0xFFU;
        SPI_I2S_SendData(SPI2, out);

        t = timeout;
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
        {
            if (t-- == 0U)
            {
                GPIO_SetBits(SPI2_GPIO_PORT, SPI2_CS_PIN);
                return SPI2_TIMEOUT;
            }
        }

        uint8_t in = (uint8_t)SPI_I2S_ReceiveData(SPI2);
        if (rx != 0)
        {
            *rx++ = in;
        }
    }

    /* Deassert CS */
    GPIO_SetBits(SPI2_GPIO_PORT, SPI2_CS_PIN);

    /* Wait until not busy */
    timeout = spi2_timeout_value(timeout);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET)
    {
        if (timeout-- == 0U)
        {
            return SPI2_TIMEOUT;
        }
    }

    return SPI2_OK;
}

SPI2_Status spi2_write(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    return spi2_transfer(data, 0, len, timeout);
}

SPI2_Status spi2_read(uint8_t *data, uint16_t len, uint32_t timeout)
{
    return spi2_transfer(0, data, len, timeout);
}


