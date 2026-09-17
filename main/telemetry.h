#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define MIRILLA_TELEMETRY_HEADER_DEVICE "X-Mirilla-Device"
#define MIRILLA_TELEMETRY_HEADER_BOARD  "X-Mirilla-Board"

#define MIRILLA_TELEMETRY_HEADER_MAX 288

typedef struct {
    float fps;
    uint32_t capture_ms;
    uint32_t upload_ms;
    uint32_t frames_ok;
    uint32_t frames_failed;
} mirilla_telemetry_loop_t;

esp_err_t mirilla_telemetry_init(void);

void mirilla_telemetry_set_loop(const mirilla_telemetry_loop_t *loop);

size_t mirilla_telemetry_device(char *out, size_t len);

size_t mirilla_telemetry_board(char *out, size_t len);
