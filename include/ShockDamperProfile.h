#pragma once

#include "config.h"

#ifndef _ARM_MATH_H
    #include "arm_math.h"
#endif

struct ShockDamperProfile
{
    float32_t PID_P;
    float32_t PID_I;
    float32_t PID_D;
};

void SdpInit(struct ShockDamperProfile *newProfilesArr, int len);

int sdpeGet(int index, struct ShockDamperProfile *requestedProfile);

int sdpUpdateProfile(int index, struct ShockDamperProfile newProfileCoefs);