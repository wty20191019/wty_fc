#include "i2c1.h"
#include "stm32f4xx_conf.h"


#define I2C1_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define I2C1_GPIO_PORT         GPIOB
#define I2C1_SCL_PIN           GPIO_Pin_6
#define I2C1_SDA_PIN           GPIO_Pin_7

#define I2C1_PERIPH_CLK        RCC_APB1Periph_I2C1
#define I2C1_TIMEOUT_DEFAULT   20000U//默认I2C通信超时时间，单位毫秒

static uint32_t i2c1_timeout_value(uint32_t timeout)
{
    return (timeout == 0U) ? I2C1_TIMEOUT_DEFAULT : timeout;
}

static void i2c1_clear_addr_flag(void)
{
    __IO uint32_t temp;

    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;
}

static void i2c1_abort(void)
{
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

static I2C1_Status i2c1_wait_flag(uint32_t flag, FlagStatus status, uint32_t timeout)
{
    timeout = i2c1_timeout_value(timeout);

    while (timeout--)
    {
        if (I2C_GetFlagStatus(I2C1, flag) == status)
        {
            return I2C1_OK;
        }

        if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF) == SET)
        {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            return I2C1_TIMEOUT;
        }
    }

    return I2C1_TIMEOUT;
}

void i2c1_init(uint32_t clock_speed)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_AHB1PeriphClockCmd(I2C1_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(I2C1_PERIPH_CLK, ENABLE);

    GPIO_PinAFConfig(I2C1_GPIO_PORT, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2C1_GPIO_PORT, GPIO_PinSource7, GPIO_AF_I2C1);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(I2C1_GPIO_PORT, &GPIO_InitStructure);

    I2C_DeInit(I2C1);
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = clock_speed;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

I2C1_Status i2c1_write(uint8_t dev_addr, const uint8_t *data, uint16_t len, uint32_t timeout)
{
    if ((data == 0) || (len == 0U))
    {
        return I2C1_ERROR;
    }

    timeout = i2c1_timeout_value(timeout);
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);

    if (i2c1_wait_flag(I2C_FLAG_BUSY, RESET, timeout) != I2C1_OK)
    {
        return I2C1_TIMEOUT;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    if (i2c1_wait_flag(I2C_FLAG_SB, SET, timeout) != I2C1_OK)
    {
        i2c1_abort();
        return I2C1_TIMEOUT;
    }

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Transmitter);
    if (i2c1_wait_flag(I2C_FLAG_ADDR, SET, timeout) != I2C1_OK)
    {
        i2c1_abort();
        return I2C1_TIMEOUT;
    }

    i2c1_clear_addr_flag();

    while (len--)
    {
        I2C_SendData(I2C1, *data++);
        if (i2c1_wait_flag(I2C_FLAG_BTF, SET, timeout) != I2C1_OK)
        {
            i2c1_abort();
            return I2C1_TIMEOUT;
        }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return I2C1_OK;
}

I2C1_Status i2c1_read(uint8_t dev_addr, uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint16_t index = 0;

    if ((data == 0) || (len == 0U))
    {
        return I2C1_ERROR;
    }

    timeout = i2c1_timeout_value(timeout);
    I2C_ClearFlag(I2C1, I2C_FLAG_AF);

    if (i2c1_wait_flag(I2C_FLAG_BUSY, RESET, timeout) != I2C1_OK)
    {
        return I2C1_TIMEOUT;
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);
    if (i2c1_wait_flag(I2C_FLAG_SB, SET, timeout) != I2C1_OK)
    {
        i2c1_abort();
        return I2C1_TIMEOUT;
    }

    I2C_Send7bitAddress(I2C1, dev_addr << 1, I2C_Direction_Receiver);
    if (i2c1_wait_flag(I2C_FLAG_ADDR, SET, timeout) != I2C1_OK)
    {
        i2c1_abort();
        return I2C1_TIMEOUT;
    }

    if (len == 1U)
    {
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        i2c1_clear_addr_flag();
        I2C_GenerateSTOP(I2C1, ENABLE);

        if (i2c1_wait_flag(I2C_FLAG_RXNE, SET, timeout) != I2C1_OK)
        {
            i2c1_abort();
            return I2C1_TIMEOUT;
        }

        data[0] = I2C_ReceiveData(I2C1);
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        return I2C1_OK;
    }

    if (len == 2U)
    {
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        i2c1_clear_addr_flag();

        if (i2c1_wait_flag(I2C_FLAG_BTF, SET, timeout) != I2C1_OK)
        {
            i2c1_abort();
            return I2C1_TIMEOUT;
        }

        I2C_GenerateSTOP(I2C1, ENABLE);
        data[0] = I2C_ReceiveData(I2C1);
        data[1] = I2C_ReceiveData(I2C1);
        I2C_AcknowledgeConfig(I2C1, ENABLE);
        return I2C1_OK;
    }

    i2c1_clear_addr_flag();

    while (len > 3U)
    {
        if (i2c1_wait_flag(I2C_FLAG_RXNE, SET, timeout) != I2C1_OK)
        {
            i2c1_abort();
            return I2C1_TIMEOUT;
        }

        data[index++] = I2C_ReceiveData(I2C1);
        len--;
    }

    if (i2c1_wait_flag(I2C_FLAG_BTF, SET, timeout) != I2C1_OK)
    {
        i2c1_abort();
        return I2C1_TIMEOUT;
    }

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    data[index++] = I2C_ReceiveData(I2C1);
    len--;

    if (i2c1_wait_flag(I2C_FLAG_BTF, SET, timeout) != I2C1_OK)
    {
        i2c1_abort();
        return I2C1_TIMEOUT;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    data[index++] = I2C_ReceiveData(I2C1);
    data[index++] = I2C_ReceiveData(I2C1);
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return I2C1_OK;
}

I2C1_Status i2c1_mem_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint8_t buffer[33];
    uint16_t index;

    if (len > 32U)
    {
        return I2C1_TIMEOUT;
    }

    buffer[0] = reg_addr;

    if ((data != 0) && (len != 0U))
    {
        for (index = 0; index < len; index++)
        {
            buffer[index + 1U] = data[index];
        }

        return i2c1_write(dev_addr, buffer, (uint16_t)(len + 1U), timeout);
    }

    return i2c1_write(dev_addr, buffer, 1U, timeout);
}

I2C1_Status i2c1_mem_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len, uint32_t timeout)
{
    uint8_t reg = reg_addr;

    if ((data == 0) || (len == 0U))
    {
        return I2C1_ERROR;
    }

    if (i2c1_write(dev_addr, &reg, 1U, timeout) != I2C1_OK)
    {
        return I2C1_TIMEOUT;
    }

    return i2c1_read(dev_addr, data, len, timeout);
}



