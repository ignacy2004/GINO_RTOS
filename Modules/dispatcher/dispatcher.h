#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "serial_interface.h"

void Dispatcher_ProcessCommand(CommandFrame_t* frame);
void Dispatcher_MoveJoints(int32_t j1_steps, int32_t j2_steps, int32_t j3_steps);
void Dispatcher_HomeAllAxes(void);
void Dispatcher_SetTeachMode(uint8_t axisId, bool releaseDrive);
void Dispatcher_StartTeachInReturnAll(void);
void Dispatcher_StopAllMotors(void);

#endif