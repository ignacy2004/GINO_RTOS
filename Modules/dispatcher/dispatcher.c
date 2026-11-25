#include "dispatcher.h"
#include <stdio.h>
#include <math.h>
#include "system_state.h"
#include "inverse_kinematics.h"
#include "tool_controller.h"
#include "config.h"

void Dispatcher_ProcessCommand(CommandFrame_t* frame) {
    if (SystemState_IsEstop() && frame->cmd != CMD_ESTOP && frame->cmd != CMD_ESTOP_LEGACY && frame->cmd != CMD_HOME) {
        printf("[DISPATCHER] E-STOP is active! Ignoring command 0x%02X\r\n", frame->cmd);
        return;
    }

    switch(frame->cmd) {
        case CMD_JOINT_MOVE: {
            int32_t j1_steps = (int32_t)lroundf(frame->param1 * STEPS_PER_DEG_J1);
            int32_t j2_steps = (int32_t)lroundf(frame->param2 * STEPS_PER_DEG_J2);
            int32_t j3_steps = (int32_t)lroundf(frame->param3 * STEPS_PER_MM_Z);
            Dispatcher_MoveJoints(j1_steps, j2_steps, j3_steps);
            SystemState_SetPendingMotionAck(frame->cmd);
            break;
        }

        case CMD_LINEAR_MOVE: {
            Point3D_t target = {
                .x = frame->param1,
                .y = frame->param2,
                .z = frame->param3
            };
            JointSteps_t targetSteps;
            if (Kinematics_CartesianToSteps(target, &targetSteps)) {
                Dispatcher_MoveJoints(targetSteps.j1_steps, targetSteps.j2_steps, targetSteps.j3_steps);
                SystemState_SetPendingMotionAck(frame->cmd);
            } else {
                // Punkt poza zasięgiem -> odsyłamy ACK ze statusem błędu (param2 = 0.0f)
                Serial_SendFrame(CMD_ACK, (float)frame->cmd, 0.0f, 0.0f);
            }
            break;
        }

        case CMD_HOME: {
            SystemState_SetEstop(false);
            SystemState_SetHomed(false);
            Dispatcher_HomeAllAxes();
            SystemState_SetPendingMotionAck(frame->cmd);
            break;
        }

        case CMD_EFFECTOR_HMI:
        case CMD_EFFECTOR_ACTION: {
            bool enablePump = (frame->param2 > 0.5f);
            ToolController_EnablePump(enablePump);
            uint32_t durationMs = (frame->param3 > 0.0f) ? (uint32_t)lroundf(frame->param3) : 0;
            if (durationMs > 0) {
                SystemState_SetPendingDelay(frame->cmd, durationMs);
            } else {
                Serial_SendFrame(CMD_ACK, (float)frame->cmd, 1.0f, 0.0f);
            }
            break;
        }

        case CMD_DELAY: {
            uint32_t delayMs = (frame->param1 > 0.0f) ? (uint32_t)lroundf(frame->param1) : 0;
            if (delayMs > 0) {
                SystemState_SetPendingDelay(frame->cmd, delayMs);
            } else {
                Serial_SendFrame(CMD_ACK, (float)frame->cmd, 1.0f, 0.0f);
            }
            break;
        }

        case CMD_TEACH_MODE: {
            uint8_t axisId = (uint8_t)lroundf(frame->param1);
            bool releaseDrive = (frame->param2 > 0.5f);
            Dispatcher_SetTeachMode(axisId, releaseDrive);
            Serial_SendFrame(CMD_ACK, (float)frame->cmd, 1.0f, 0.0f);
            break;
        }

        case CMD_TEACH_READ: {
            SystemState_SetEstop(false);
            SystemState_SetHomed(false);
            Dispatcher_StartTeachInReturnAll();
            break;
        }

        case CMD_ESTOP:
        case CMD_ESTOP_LEGACY: {
            SystemState_SetEstop(true);
            SystemState_SetPendingMotionAck(0);
            ToolController_EnablePump(false);
            Dispatcher_StopAllMotors();
            break;
        }

        default:
            printf("[DISPATCHER] Unknown cmd: 0x%02X\r\n", frame->cmd);
            break;
    }
}