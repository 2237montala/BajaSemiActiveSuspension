#pragma once

#include "arm_math.h"

enum PREVIOUS_PID_DATA_INDEX {PID_PREV_ERROR = 0, PID_PREV_I = 1, PID_PREV_COUNT = 2};


struct pid_instance_f32_struct {
    float32_t prevData[PID_PREV_COUNT];    
    float32_t Kp;          /**< The proportional gain. */
    float32_t Ki;          /**< The integral gain. */
    float32_t Kd;          /**< The derivative gain. */
};

void PidInstanceInitF32(struct pid_instance_f32_struct *s);

void PidInstanceSetParamsF32(struct pid_instance_f32_struct *s, float32_t p, float32_t i, float32_t d);

void PidInstanceResetF32(struct pid_instance_f32_struct *s);

float32_t PidComputeF32(struct pid_instance_f32_struct *s, float32_t pidError, float32_t sampleTime);

void PidInstanceSetInitPrevData(struct pid_instance_f32_struct *s, float32_t prevData[PID_PREV_COUNT]);