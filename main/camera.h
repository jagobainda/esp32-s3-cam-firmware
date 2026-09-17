#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_camera.h"

typedef struct {
    bool valid;
    uint16_t exposure_ms;
    uint16_t ceiling_ms;
    uint16_t frame_ms;
    uint16_t gain_x100;
    uint8_t avg;
} mirilla_camera_aec_t;

esp_err_t mirilla_camera_init(void);

const char *mirilla_camera_resolution_str(void);

void mirilla_camera_keep_exposure_policy(void);

void mirilla_camera_benchmark(int seconds);

void mirilla_camera_aec(mirilla_camera_aec_t *out);
