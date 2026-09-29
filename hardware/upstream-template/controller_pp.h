#pragma once
#include "stabilizer_types.h"
#include "controller.h"

void controllerPPInit(void);
bool controllerPPTest(void);
void controllerPP(control_t *control, const setpoint_t *setpoint,
                  const sensorData_t *sensors, const state_t *state,
                  const stabilizerStep_t stabilizerStep);
