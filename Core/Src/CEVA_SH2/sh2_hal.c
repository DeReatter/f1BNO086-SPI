/*
 * sh2_hal.c
 *
 * Driver HAL interface for BNO080/BNO085/BNO086 SPI protocol on STM32F1
 */

#include "CEVA_SH2/sh2_hal.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

// Global interrupt flag for BNO086 INT pin
volatile uint8_t bno_int_triggered = 0;

// Buffer dummy 0x00 untuk clocking penerimaan payload SPI
static uint8_t dummy_zeros[SH2_HAL_MAX_TRANSFER_IN] = {0};

/**
 * @brief EXTI callback when BNO086 INT pin goes LOW
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BNO_INT_Pin) {
        bno_int_triggered = 1;
    }
}

/**
 * @brief Initialize DWT cycle counter on Cortex-M3 (for high-precision microsecond timing)
 */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Provide microsecond timestamp with sub-millisecond precision
 */
static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    uint32_t ms, st;
    do {
        ms = HAL_GetTick();
        st = SysTick->VAL;
    } while (ms != HAL_GetTick());

    uint32_t ticks_per_ms = SystemCoreClock / 1000U;
    uint32_t elapsed_ticks = (SysTick->LOAD > st) ? (SysTick->LOAD - st) : 0;
    uint32_t us = (elapsed_ticks * 1000U) / ticks_per_ms;
    return (ms * 1000U) + us;
}

/**
 * @brief Open SH2 communication: hardware reset sequence into SPI mode
 */
static int hal_open(sh2_Hal_t *self) {
    dwt_init();

    // 1. Set pin kontrol ke idle
    HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);
    // BNO_WAK (PS0) WAJIB HIGH saat reset dilepas agar BNO boot ke mode SPI
    HAL_GPIO_WritePin(BNO_WAK_GPIO_Port, BNO_WAK_Pin, GPIO_PIN_SET);

    // 2. Hardware Reset BNO086
    HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(150); // Tunggu bootloader internal BNO086 selesai inisialisasi

    // 3. Bersihkan interrupt liar (phantom interrupt) selama masa reset
    __HAL_GPIO_EXTI_CLEAR_IT(BNO_INT_Pin);
    bno_int_triggered = 0;

    // 4. Periksa apakah sensor sudah menarik INT ke LOW
    if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET) {
        bno_int_triggered = 1;
    }

    return 0;
}

/**
 * @brief Close SH2 communication
 */
static void hal_close(sh2_Hal_t *self) {
    HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BNO_WAK_GPIO_Port, BNO_WAK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Read an SHTP packet from BNO086 over SPI
 */
static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    // Fallback: Jika INT sudah LOW secara fisik, set flag
    if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET) {
        bno_int_triggered = 1;
    }

    if (!bno_int_triggered) {
        return 0;
    }

    uint8_t header[4] = {0};
    uint8_t dummy[4] = {0};

    // Tarik CS LOW
    HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_RESET);

    // 1. Baca 4 byte header SHTP (kirim dummy 0x00)
    if (HAL_SPI_TransmitReceive(&hspi1, dummy, header, 4, 10) != HAL_OK) {
        HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);
        bno_int_triggered = 0;
        return 0;
    }

    uint16_t packet_len = (uint16_t)(header[0] | (header[1] << 8)) & ~0x8000;

    // Validasi panjang paket: jika 0 atau 0x7FFF (SPI noise/tidak ada data) atau melebihi len
    if (packet_len == 0 || packet_len == 0x7FFF || packet_len > len) {
        HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);
        bno_int_triggered = 0;
        return 0;
    }

    // Salin 4 byte header ke pBuffer
    for (uint8_t i = 0; i < 4; i++) {
        pBuffer[i] = header[i];
    }

    // 2. Baca seluruh payload (jika packet_len > 4)
    if (packet_len > 4) {
        uint16_t payload_len = packet_len - 4;
        if (HAL_SPI_TransmitReceive(&hspi1, dummy_zeros, pBuffer + 4, payload_len, 50) != HAL_OK) {
            HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);
            bno_int_triggered = 0;
            return 0;
        }
    }

    // Lepaskan CS HIGH
    HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

    bno_int_triggered = 0;
    *t_us = hal_getTimeUs(self);

    return packet_len;
}

/**
 * @brief Write an SHTP packet to BNO086 over SPI
 *
 * Sequence:
 * 1. Assert WAKE LOW to request write.
 * 2. Wait until BNO086 asserts INT LOW (ready to receive).
 * 3. Assert CS LOW.
 * 4. Transmit data via SPI.
 * 5. Deassert CS HIGH.
 * 6. Deassert WAKE HIGH.
 */
static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    // 1. Tarik WAKE ke LOW untuk membangunkan sensor
    HAL_GPIO_WritePin(BNO_WAK_GPIO_Port, BNO_WAK_Pin, GPIO_PIN_RESET);

    // 2. Tunggu sampai INT menjadi LOW dari sensor
    uint32_t start_time = hal_getTimeUs(self);
    while (!bno_int_triggered) {
        if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_RESET) {
            bno_int_triggered = 1;
            break;
        }

        // Timeout 15 ms
        if ((hal_getTimeUs(self) - start_time) > 15000) {
            HAL_GPIO_WritePin(BNO_WAK_GPIO_Port, BNO_WAK_Pin, GPIO_PIN_SET);
            return 0; // SHTP layer akan mengulang (retry)
        }
    }

    // 3. Tarik CS LOW
    HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_RESET);

    // 4. Kirim paket SHTP
    HAL_StatusTypeDef tx_status = HAL_SPI_Transmit(&hspi1, pBuffer, len, 50);

    // 5. Lepas CS HIGH
    HAL_GPIO_WritePin(BNO_CS_GPIO_Port, BNO_CS_Pin, GPIO_PIN_SET);

    // 6. Lepas WAKE HIGH
    HAL_GPIO_WritePin(BNO_WAK_GPIO_Port, BNO_WAK_Pin, GPIO_PIN_SET);

    bno_int_triggered = 0;

    return (tx_status == HAL_OK) ? len : 0;
}

// Instance struct HAL untuk didaftarkan ke SH2
sh2_Hal_t my_sh2_hal = {
    .open = hal_open,
    .close = hal_close,
    .read = hal_read,
    .write = hal_write,
    .getTimeUs = hal_getTimeUs
};

sh2_Hal_t *get_sh2_hal(void) {
    return &my_sh2_hal;
}
