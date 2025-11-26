#ifndef DIPAN_H
#define DIPAN_H

#include "main.h"
#include "motor.h"
#include "pid.h"

void Dipan_Init(void);

uint8_t Dipan_Mecanum(float vx, float vy, float w);

#endif
