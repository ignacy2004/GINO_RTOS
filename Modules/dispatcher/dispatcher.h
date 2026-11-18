#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "serial_interface.h"

void Dispatcher_ProcessCommand(CommandFrame_t* frame);
void Dispatcher_MoveJoints(int32_t j1_steps, int32_t j2_steps, int32_t j3_steps);
void Dispatcher_HomeMotors(uint8_t axisId);
void Dispatcher_StopAllMotors(void);

#endif