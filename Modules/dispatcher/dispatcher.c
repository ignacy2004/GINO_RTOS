#include "dispatcher.h"
#include <stdio.h>
#include <math.h>
#include "system_state.h"
#include "inverse_kinematics.h"
#include "tool_controller.h"

void Dispatcher_ProcessCommand(CommandFrame_t* frame) {
    if (SystemState_IsEstop() && frame->cmd != CMD_ESTOP && frame->cmd != CMD_ESTOP_LEGACY) {
        printf("[DISPATCHER] E-STOP is active! Ignoring command 0x%02X\r\n", frame->cmd);
        return;
    }

    switch(frame->cmd) {
        case CMD_JOINT_MOVE: {
            int32_t j1_steps = (int32_t)lroundf(frame->param1 * STEPS_PER_DEG_J1);
            int32_t j2_steps = (int32_t)lroundf(frame->param2 * STEPS_PER_DEG_J2);
            int32_t j3_steps = (int32_t)lroundf(frame->param3 * STEPS_PER_MM_Z);
            Dispatcher_MoveJoints(j1_steps, j2_steps, j3_steps);
            break;
        }

        case CMD_LINEAR_MOVE: {
            printf("[DISPATCHER] CMD_LINEAR_MOVE -> X:%.2f Y:%.2f Z:%.2f\r\n", 
                   frame->param1, frame->param2, frame->param3);
            Point3D_t target = {
                .x = frame->param1,
                .y = frame->param2,
                .z = frame->param3
            };
            JointSteps_t targetSteps;
            if (Kinematics_CartesianToSteps(target, &targetSteps)) {
                Dispatcher_MoveJoints(targetSteps.j1_steps, targetSteps.j2_steps, targetSteps.j3_steps);
            } else {
                printf("[DISPATCHER] Linear target unreachable!\r\n");
            }
            break;
        }

        case CMD_EFFECTOR_ACTION: {
            bool enablePump = (frame->param2 > 0.5f);
            printf("[DISPATCHER] CMD_EFFECTOR_ACTION -> Tool: %.0f, State: %s\r\n", 
                   frame->param1, enablePump ? "ON" : "OFF");
            ToolController_EnablePump(enablePump);
            break;
        }

        case CMD_ESTOP:
        case CMD_ESTOP_LEGACY: {
            printf("[DISPATCHER] *** EMERGENCY STOP (ESTOP) ***\r\n");
            SystemState_SetEstop(true);
            ToolController_EnablePump(false);
            Dispatcher_StopAllMotors();
            break;
        }

        default:
            printf("[DISPATCHER] Unknown cmd: 0x%02X\r\n", frame->cmd);
            break;
    }
}