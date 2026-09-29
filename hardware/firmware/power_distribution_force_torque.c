/**
 * @file power_distribution_force_torque.c
 * @brief Custom force/torque allocation logic used during Crazyflie hardware validation.
 *
 * This is the project-specific portion reconstructed from the modified
 * Bitcraze power_distribution_quadrotor.c listing in the final report.
 *
 * The surrounding stock Crazyflie firmware is Copyright (C) Bitcraze AB
 * and distributed under GPL-3.0. This file is provided as a reference patch,
 * not as a standalone replacement for the complete upstream source file.
 */

#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform_defaults.h"

#include <math.h>
#include <stdint.h>

static float armLength = ARM_LENGTH;
static float thrustToTorque = 0.005964552f;

static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;

static float log_pwm1;
static float log_pwm2;
static float log_pwm3;
static float log_pwm4;

static float log_alpha;
static float log_Tdes;
static float log_Ldes;
static float log_Mdes;
static float log_Ndes;

static float clampFloatCustom(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static bool thrustIsFeasible(const float T_motor[4], float T_min, float T_max) {
  for (int i = 0; i < 4; i++) {
    if (T_motor[i] < T_min || T_motor[i] > T_max) {
      return false;
    }
  }
  return true;
}

static void mixerSolve(
  float T_des,
  float L_des,
  float M_des,
  float N_des,
  float T_motor[4]
) {
  const float l_eff = armLength / sqrtf(2.0f);
  const float k = thrustToTorque;

  T_motor[0] = 0.25f * (T_des - L_des / l_eff - M_des / l_eff + N_des / k);
  T_motor[1] = 0.25f * (T_des - L_des / l_eff + M_des / l_eff - N_des / k);
  T_motor[2] = 0.25f * (T_des + L_des / l_eff + M_des / l_eff + N_des / k);
  T_motor[3] = 0.25f * (T_des + L_des / l_eff - M_des / l_eff - N_des / k);
}

static uint16_t thrustToPWMCustom(float thrust) {
  const float A = pwmToThrustA;
  const float B = pwmToThrustB;
  const float PWM_min = 0.0f;
  const float PWM_max = 1.0f;

  thrust = clampFloatCustom(thrust, 0.0f, A + B);

  const float delta = B * B + 4.0f * A * thrust;
  float PWM = 0.0f;

  if (delta >= 0.0f) {
    PWM = (-B + sqrtf(delta)) / (2.0f * A);
  }

  PWM = clampFloatCustom(PWM, PWM_min, PWM_max);
  return (uint16_t)(PWM * 65535.0f + 0.5f);
}

/*
 * Merge this function into the stock Bitcraze power-distribution implementation
 * as the controlModeForceTorque branch.
 */
static void powerDistributionForceTorque(
  const control_t *control,
  motors_thrust_uncapped_t *motorThrustUncapped
) {
  float T_des = control->thrustSi;
  const float L_des = control->torqueX;
  const float M_des = control->torqueY;
  const float N_des = control->torqueZ;

  float alpha_used = 1.0f;

  const float A = pwmToThrustA;
  const float B = pwmToThrustB;
  const float PWM_min = 0.0f;
  const float PWM_max = 1.0f;

  const float T_min = A * PWM_min * PWM_min + B * PWM_min;
  const float T_max = A * PWM_max * PWM_max + B * PWM_max;

  const float T_total_min = 4.0f * T_min;
  const float T_total_max = 4.0f * T_max;

  if (T_des < T_total_min || T_des > T_total_max) {
    T_des = clampFloatCustom(T_des, T_total_min, T_total_max);
  }

  float T_motor[4];
  mixerSolve(T_des, L_des, M_des, N_des, T_motor);

  if (!thrustIsFeasible(T_motor, T_min, T_max)) {
    for (int alpha_index = 1000; alpha_index >= 0; alpha_index--) {
      const float alpha = ((float)alpha_index) / 1000.0f;

      mixerSolve(
        T_des,
        alpha * L_des,
        alpha * M_des,
        alpha * N_des,
        T_motor
      );

      if (thrustIsFeasible(T_motor, T_min, T_max)) {
        alpha_used = alpha;
        break;
      }
    }
  }

  uint16_t pwm[4];
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    pwm[motorIndex] = thrustToPWMCustom(T_motor[motorIndex]);
    motorThrustUncapped->list[motorIndex] = pwm[motorIndex];
  }

  log_pwm1 = (float)pwm[0];
  log_pwm2 = (float)pwm[1];
  log_pwm3 = (float)pwm[2];
  log_pwm4 = (float)pwm[3];

  log_alpha = alpha_used;
  log_Tdes = T_des;
  log_Ldes = L_des;
  log_Mdes = M_des;
  log_Ndes = N_des;
}

LOG_GROUP_START(ppMix)
LOG_ADD(LOG_FLOAT, pwm1, &log_pwm1)
LOG_ADD(LOG_FLOAT, pwm2, &log_pwm2)
LOG_ADD(LOG_FLOAT, pwm3, &log_pwm3)
LOG_ADD(LOG_FLOAT, pwm4, &log_pwm4)
LOG_ADD(LOG_FLOAT, alpha, &log_alpha)
LOG_ADD(LOG_FLOAT, Tdes, &log_Tdes)
LOG_ADD(LOG_FLOAT, Ldes, &log_Ldes)
LOG_ADD(LOG_FLOAT, Mdes, &log_Mdes)
LOG_ADD(LOG_FLOAT, Ndes, &log_Ndes)
LOG_GROUP_STOP(ppMix)

PARAM_GROUP_START(quadSysId)
PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_GROUP_STOP(quadSysId)
