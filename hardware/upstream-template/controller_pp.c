/**
 * @file controller_pp.c
 * @brief Pole-Placement Controller Implementation
 * @author Will Wu, Brown Unniversity/Washington University in St. Louis, 2025
 */

#include "controller_pp.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"

static bool is_init = false;
// global controller functions
void controllerPPInit(void) {
    if (is_init) {
        return;
    }
    is_init = true;
}

bool controllerPPTest(void) {
    return true;
}

#define UPDATE_RATE RATE_500_HZ

void controllerPP(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep) {
    // limits controller rate to 500Hz.
    if (!RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
        return;
    }
    // TODO: put your controller code below

    if (setpoint->mode.z == modeDisable) {
        // Disarm motors when the drone is on the ground/landed
        control->thrustSi = 0.0f;
        control->torqueX = 0.0f;
        control->torqueY = 0.0f;
        control->torqueZ = 0.0f;
    } else {
        // TODO: send command to mixer below
    }
    control->controlMode = controlModeForceTorque; // use custom mixer
}

LOG_GROUP_START(PP)
LOG_GROUP_STOP(PP)

PARAM_GROUP_START(PP)
PARAM_GROUP_STOP(PP)