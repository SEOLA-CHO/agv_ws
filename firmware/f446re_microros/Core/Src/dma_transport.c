#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "app_config.h"
#include "cmsis_os.h"
#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static UART_HandleTypeDef *transport_uart;
static volatile uint32_t rx_wrap_count;
static uint32_t rx_consumed_total;
static volatile bool rx_restart_requested;
static volatile uint32_t uart_transport_error_count;

static void set_error(uint8_t *error_code)
{
    if (error_code != NULL) {
        *error_code = 1U;
    }
    ++uart_transport_error_count;
}

static void clear_error(uint8_t *error_code)
{
    if (error_code != NULL) {
        *error_code = 0U;
    }
}

static bool start_rx_dma(UART_HandleTypeDef *uart)
{
    HAL_StatusTypeDef result;

    if ((uart == NULL) || (uart->hdmarx == NULL)) {
        return false;
    }

    rx_wrap_count = 0U;
    rx_consumed_total = 0U;
    rx_restart_requested = false;
    result = HAL_UART_Receive_DMA(
        uart, dma_buffer, (uint16_t)UART_DMA_BUFFER_SIZE);
    return result == HAL_OK;
}

static bool recover_rx_dma_if_needed(UART_HandleTypeDef *uart)
{
    if (!rx_restart_requested) {
        return true;
    }

    (void)HAL_UART_DMAStop(uart);
    __HAL_UART_CLEAR_OREFLAG(uart);
    if (!start_rx_dma(uart)) {
        rx_restart_requested = true;
        return false;
    }
    return true;
}

static uint32_t rx_produced_total(UART_HandleTypeDef *uart)
{
    uint32_t primask;
    uint32_t wraps;
    uint32_t position;
    uint32_t produced;

    primask = __get_PRIMASK();
    __disable_irq();
    wraps = rx_wrap_count;
    position = UART_DMA_BUFFER_SIZE -
               __HAL_DMA_GET_COUNTER(uart->hdmarx);
    if (primask == 0U) {
        __enable_irq();
    }

    produced = (wraps * UART_DMA_BUFFER_SIZE) + position;
    /* The DMA counter can reload just before its completion ISR increments
       rx_wrap_count. Account for that short, observable pending-wrap window. */
    if ((int32_t)(produced - rx_consumed_total) < 0) {
        produced += UART_DMA_BUFFER_SIZE;
    }
    return produced;
}

bool cubemx_transport_open(struct uxrCustomTransport *transport)
{
    UART_HandleTypeDef *uart;

    if ((transport == NULL) || (transport->args == NULL)) {
        ++uart_transport_error_count;
        return false;
    }

    uart = (UART_HandleTypeDef *)transport->args;
    transport_uart = uart;
    (void)HAL_UART_DMAStop(uart);
    if (!start_rx_dma(uart)) {
        ++uart_transport_error_count;
        return false;
    }
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport)
{
    UART_HandleTypeDef *uart;
    HAL_StatusTypeDef result;

    if ((transport == NULL) || (transport->args == NULL)) {
        ++uart_transport_error_count;
        return false;
    }

    uart = (UART_HandleTypeDef *)transport->args;
    result = HAL_UART_DMAStop(uart);
    transport_uart = NULL;
    return result == HAL_OK;
}

size_t cubemx_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buffer,
    size_t length,
    uint8_t *error_code)
{
    UART_HandleTypeDef *uart;
    HAL_StatusTypeDef result;
    uint32_t started_at;

    clear_error(error_code);
    if ((transport == NULL) || (transport->args == NULL) ||
        (buffer == NULL) || (length == 0U) ||
        (length > UINT16_MAX)) {
        set_error(error_code);
        return 0U;
    }

    uart = (UART_HandleTypeDef *)transport->args;
    if (uart->gState != HAL_UART_STATE_READY) {
        set_error(error_code);
        return 0U;
    }

    result = HAL_UART_Transmit_DMA(
        uart, (uint8_t *)buffer, (uint16_t)length);
    if (result != HAL_OK) {
        set_error(error_code);
        return 0U;
    }

    started_at = HAL_GetTick();
    while (uart->gState != HAL_UART_STATE_READY) {
        if ((uint32_t)(HAL_GetTick() - started_at) >=
            UART_TX_TIMEOUT_MS) {
            (void)HAL_UART_AbortTransmit(uart);
            set_error(error_code);
            return 0U;
        }
        osDelay(1U);
    }

    return length;
}

size_t cubemx_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buffer,
    size_t length,
    int timeout_ms,
    uint8_t *error_code)
{
    UART_HandleTypeDef *uart;
    uint32_t started_at;
    uint32_t produced;
    uint32_t available;
    size_t copied = 0U;

    clear_error(error_code);
    if ((transport == NULL) || (transport->args == NULL) ||
        (buffer == NULL) || (length == 0U)) {
        set_error(error_code);
        return 0U;
    }

    uart = (UART_HandleTypeDef *)transport->args;
    if (!recover_rx_dma_if_needed(uart)) {
        set_error(error_code);
        return 0U;
    }

    started_at = HAL_GetTick();
    do {
        produced = rx_produced_total(uart);
        available = produced - rx_consumed_total;
        if (available != 0U) {
            break;
        }
        if (timeout_ms <= 0) {
            return 0U;
        }
        osDelay(1U);
    } while ((uint32_t)(HAL_GetTick() - started_at) <
             (uint32_t)timeout_ms);

    if (available > UART_DMA_BUFFER_SIZE) {
        rx_consumed_total = produced - UART_DMA_BUFFER_SIZE;
        available = UART_DMA_BUFFER_SIZE;
        set_error(error_code);
    }

    while ((copied < length) && (copied < available)) {
        buffer[copied] =
            dma_buffer[rx_consumed_total % UART_DMA_BUFFER_SIZE];
        ++rx_consumed_total;
        ++copied;
    }
    return copied;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if ((uart != NULL) && (uart == transport_uart)) {
        ++rx_wrap_count;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if ((uart != NULL) && (uart == transport_uart)) {
        ++uart_transport_error_count;
        rx_restart_requested = true;
    }
}

#endif /* RMW_UXRCE_TRANSPORT_CUSTOM */
