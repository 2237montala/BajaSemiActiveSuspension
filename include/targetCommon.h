#pragma once
/*
 * HEADER NAME : targetCommon.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/03/2020
 * DESCRIPTION :
 *      Holds all the includes that are special for the system being compiled
 *      for
 */

#include <Bsp.h>

#define NL (char *)("\r\n")

struct ioVec {
    uintptr_t *data;
    uint32_t len;
};


