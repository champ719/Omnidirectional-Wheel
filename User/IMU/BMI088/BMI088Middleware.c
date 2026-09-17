#include "BMI088Middleware.h"
#include "bsp_dwt.h"
#include "main.h"
#include "spi.h"

#define BMI088_SPI_TIMEOUT_MS 10U

void BMI088_GPIO_init(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

void BMI088_com_init(void)
{
}

void BMI088_delay_ms(uint16_t ms)
{
    DWT_Delay((float)ms * 0.001f);
}

void BMI088_delay_us(uint16_t us)
{
    DWT_Delay((float)us * 0.000001f);
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0U;

    if (HAL_SPI_TransmitReceive(&hspi1,
                               &txdata,
                               &rxdata,
                               1U,
                               BMI088_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    return rxdata;
}
