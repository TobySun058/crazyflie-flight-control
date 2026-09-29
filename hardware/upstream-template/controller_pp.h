/**
 * @file controller_pp.h
 * @brief Pole-Placement Controller Interface
 * @author Will Wu, Brown Unniversity/Washington University in St. Louis, 2025
 */
#ifndef __CONTROLLER_PP_H__
#define __CONTROLLER_PP_H__

#include "stabilizer_types.h"
#include "math3d.h"

void controllerPPInit(void);
bool controllerPPTest(void);
void controllerPP(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);

#endif //__CONTROLLER_PP_H__