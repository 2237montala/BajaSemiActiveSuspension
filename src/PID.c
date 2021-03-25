#include "PID.h"

void PidInstanceInitF32(struct pid_instance_f32_struct *s) {
    PidInstanceResetF32(s);
}

void PidInstanceSetParamsF32(struct pid_instance_f32_struct *s, float32_t p, float32_t i, float32_t d) {
    s->Kp = p;
    s->Ki = i;
    s->Kd = d;
}

void PidInstanceResetF32(struct pid_instance_f32_struct *s) {
    memset(s, 0x0, sizeof(struct pid_instance_f32_struct));
}

float32_t PidComputeF32(struct pid_instance_f32_struct *s, float32_t pidError, float32_t sampleTime) {
    float32_t newP,newI,newD = 0;

    // Calulate the new values
    newP = s->Kp * pidError;
    newI = s->prevData[PID_PREV_I] + (s->Ki * pidError * sampleTime);
    newD = s->Kd * ((pidError - s->prevData[PID_PREV_ERROR]) / sampleTime);

    // Save the current values for the next loop
    s->prevData[PID_PREV_ERROR] = pidError;
    s->prevData[PID_PREV_I] = newI;

    return newP + newI + newD;
}

void PidInstanceSetInitPrevData(struct pid_instance_f32_struct *s, float32_t prevData[PID_PREV_COUNT]) {
    memcpy(s->prevData,prevData, sizeof(float32_t)*PID_PREV_COUNT);
}