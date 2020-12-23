#include "ShockDamperProfile.h"

struct ShockDamperProfile shockDamperProfiles[NUM_SHOCK_PROFILES];

void SdpInit(struct ShockDamperProfile *newProfilesArr, int len) {
    // Copy the newProfiles to the permanent profile array
    memcpy(shockDamperProfiles, newProfilesArr,(sizeof(struct ShockDamperProfile) * len));

}

int sdpeGet(int index, struct ShockDamperProfile *requestedProfile);

int sdpUpdateProfile(int index, struct ShockDamperProfile newProfileCoefs);