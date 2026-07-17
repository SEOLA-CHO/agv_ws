#ifndef VESC_CAN_TELEMETRY_H
#define VESC_CAN_TELEMETRY_H

#include "app_config.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float velocity_rad_s[WHEEL_COUNT];
    int32_t erpm[WHEEL_COUNT];
    int64_t tachometer_counts[WHEEL_COUNT];
    bool online[WHEEL_COUNT];
} vesc_telemetry_snapshot_t;

bool vesc_can_telemetry_init(CAN_HandleTypeDef *hcan);
void vesc_can_telemetry_get_snapshot(vesc_telemetry_snapshot_t *snapshot);

#endif /* VESC_CAN_TELEMETRY_H */
