#include "vesc_can_telemetry.h"

#include <string.h>

typedef struct {
    volatile int32_t erpm;
    volatile int32_t last_tachometer_raw;
    volatile int64_t extended_tachometer;
    volatile uint32_t status_rx_ms;
    volatile uint32_t status_5_rx_ms;
    volatile bool status_valid;
    volatile bool status_5_valid;
} vesc_telemetry_slot_t;

static CAN_HandleTypeDef *telemetry_hcan;
static vesc_telemetry_slot_t telemetry_slots[WHEEL_COUNT];
static volatile uint32_t can_rx_error_count;
static volatile uint32_t unknown_vesc_packet_count;
static volatile uint32_t invalid_vesc_id_count;

static int32_t read_int32_be(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 24) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) |
                     (uint32_t)data[3];
    return (int32_t)value;
}

static int32_t wheel_index_from_can_id(uint8_t can_id)
{
    uint32_t wheel;

    for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
        if (wheel_can_ids[wheel] == can_id) {
            return (int32_t)wheel;
        }
    }
    return -1;
}

bool vesc_can_telemetry_init(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) {
        return false;
    }

    telemetry_hcan = hcan;
    memset(telemetry_slots, 0, sizeof(telemetry_slots));
    return HAL_CAN_ActivateNotification(
        hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
}

void vesc_can_telemetry_get_snapshot(vesc_telemetry_snapshot_t *snapshot)
{
    uint32_t primask;
    uint32_t now_ms;
    uint32_t wheel;

    if (snapshot == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    now_ms = HAL_GetTick();

    for (wheel = 0U; wheel < WHEEL_COUNT; ++wheel) {
        bool online = telemetry_slots[wheel].status_valid &&
                      telemetry_slots[wheel].status_5_valid &&
                      ((uint32_t)(now_ms - telemetry_slots[wheel].status_rx_ms) <=
                       VESC_TELEMETRY_TIMEOUT_MS) &&
                      ((uint32_t)(now_ms - telemetry_slots[wheel].status_5_rx_ms) <=
                       VESC_TELEMETRY_TIMEOUT_MS);

        snapshot->online[wheel] = online;
        snapshot->tachometer_counts[wheel] =
            telemetry_slots[wheel].extended_tachometer;
        if (online) {
            snapshot->erpm[wheel] = telemetry_slots[wheel].erpm;
            snapshot->velocity_rad_s[wheel] =
                ((float)telemetry_slots[wheel].erpm /
                 ERPM_PER_RAD_S) * (float)wheel_directions[wheel];
        } else {
            snapshot->erpm[wheel] = 0;
            snapshot->velocity_rad_s[wheel] = 0.0f;
        }
    }

    if (primask == 0U) {
        __enable_irq();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((hcan == NULL) || (hcan != telemetry_hcan)) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        uint32_t packet_id;
        uint8_t can_id;
        int32_t wheel;
        uint32_t now_ms;

        if (HAL_CAN_GetRxMessage(
                hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            ++can_rx_error_count;
            return;
        }
        if ((header.IDE != CAN_ID_EXT) ||
            (header.RTR != CAN_RTR_DATA)) {
            continue;
        }

        packet_id = header.ExtId >> 8;
        if ((packet_id != CAN_PACKET_STATUS) &&
            (packet_id != CAN_PACKET_STATUS_5)) {
            ++unknown_vesc_packet_count;
            continue;
        }

        can_id = (uint8_t)(header.ExtId & 0xFFU);
        wheel = wheel_index_from_can_id(can_id);
        if (wheel < 0) {
            ++invalid_vesc_id_count;
            continue;
        }

        now_ms = HAL_GetTick();
        if (packet_id == CAN_PACKET_STATUS) {
            if (header.DLC < 4U) {
                ++can_rx_error_count;
                continue;
            }
            telemetry_slots[wheel].erpm = read_int32_be(data);
            telemetry_slots[wheel].status_rx_ms = now_ms;
            telemetry_slots[wheel].status_valid = true;
        } else {
            int32_t new_raw;

            if (header.DLC < 6U) {
                ++can_rx_error_count;
                continue;
            }
            new_raw = read_int32_be(data);
            if (!telemetry_slots[wheel].status_5_valid) {
                telemetry_slots[wheel].extended_tachometer =
                    (int64_t)new_raw;
            } else {
                int32_t delta = (int32_t)(
                    (uint32_t)new_raw -
                    (uint32_t)telemetry_slots[wheel].last_tachometer_raw);
                telemetry_slots[wheel].extended_tachometer +=
                    (int64_t)delta;
            }
            telemetry_slots[wheel].last_tachometer_raw = new_raw;
            telemetry_slots[wheel].status_5_rx_ms = now_ms;
            telemetry_slots[wheel].status_5_valid = true;
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == telemetry_hcan) {
        ++can_rx_error_count;
    }
}
