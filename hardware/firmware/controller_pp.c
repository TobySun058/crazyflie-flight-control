/**
 * @file controller_pp.c
 * @brief Pole-placement / LQR-style controller implementation used for hardware validation.
 *
 * Original course firmware template attribution preserved from the submitted report:
 * @author Will Wu, Brown University / Washington University in St. Louis, 2025
 *
 * This source was reconstructed from the code listing included in the final project report.
 */

#include "controller_pp.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"

#include <stdint.h>
#include <math.h>

static bool is_init = false;

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

static const float m_val = 0.033f;
static const float g_val = 9.81f;
static const float T_hover_total = 0.033f * 9.81f;

static const float K[4][12] = {
  // Altitude control
  {
    0.0f, 0.0f, 0.197252f,
    0.0f, 0.0f, 0.127875f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f
  },

  // Lateral position control
  {
    0.0f, -0.0000333893f, 0.0f,
    0.0f, -0.0000693810f, 0.0f,
    0.0000132933f, 0.0f, 0.0f,
    -0.0000042105f, 0.0f, 0.0f
  },

  // Forward position control
  {
    -0.0000333893f, 0.0f, 0.0f,
    -0.0000693810f, 0.0f, 0.0f,
    0.0f, 0.0000132933f, 0.0f,
    0.0f, -0.0000042105f, 0.0f
  },

  // Yaw-rate control
  {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0000010000f
  }
};

static const float Ki_z = 0.100000f;
static const float Ki_x = 0.0000077460f;
static const float Ki_y = 0.0000077460f;

static float int_z = 0.0f;
static float int_x = 0.0f;
static float int_y = 0.0f;

static float log_z;
static float log_ez;
static float log_thrust;
static float log_torqueX;
static float log_torqueY;
static float log_torqueZ;

static float clampFloatLocal(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void controllerPP(
  control_t *control,
  const setpoint_t *setpoint,
  const sensorData_t *sensors,
  const state_t *state,
  const stabilizerStep_t stabilizerStep
) {
  if (!RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
    return;
  }

  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torqueX = 0.0f;
    control->torqueY = 0.0f;
    control->torqueZ = 0.0f;

    int_z = 0.0f;
    int_x = 0.0f;
    int_y = 0.0f;

    log_z = state->position.z;
    log_ez = 0.0f;
    log_thrust = 0.0f;
    log_torqueX = 0.0f;
    log_torqueY = 0.0f;
    log_torqueZ = 0.0f;
  } else {
    float xhat[12];

    xhat[0] = state->position.x;
    xhat[1] = state->position.y;
    xhat[2] = state->position.z;

    xhat[3] = state->velocity.x;
    xhat[4] = state->velocity.y;
    xhat[5] = state->velocity.z;

    xhat[6] = state->attitude.roll;
    xhat[7] = state->attitude.pitch;
    xhat[8] = state->attitude.yaw;

    xhat[9] = sensors->gyro.x;
    xhat[10] = -sensors->gyro.y;
    xhat[11] = sensors->gyro.z;

    float r[12] = {0.0f};
    r[0] = 0.0f;
    r[1] = 0.0f;

    if (setpoint->mode.z != modeDisable) {
      r[2] = clampFloatLocal(setpoint->position.z, 0.0f, 0.50f);
    } else {
      r[2] = xhat[2];
    }

    // Hold the current yaw reference.
    r[8] = xhat[8];

    float e[12];
    for (int i = 0; i < 12; i++) {
      e[i] = r[i] - xhat[i];
    }

    const float dt = 1.0f / 500.0f;
    const bool xy_integrator_enabled = (xhat[2] > 0.15f);

    int_z += e[2] * dt;

    if (xy_integrator_enabled) {
      if (fabsf(e[0]) > 0.03f) {
        int_x += e[0] * dt;
      }
      if (fabsf(e[1]) > 0.03f) {
        int_y += e[1] * dt;
      }
    } else {
      int_x = 0.0f;
      int_y = 0.0f;
    }

    int_x = clampFloatLocal(int_x, -0.5f, 0.5f);
    int_y = clampFloatLocal(int_y, -0.5f, 0.5f);
    int_z = clampFloatLocal(int_z, -1.0f, 1.0f);

    /*
     * These resets are preserved from the submitted final-report listing.
     * They make the integral contribution zero in this recorded version.
     */
    int_x = 0.0f;
    int_y = 0.0f;
    int_z = 0.0f;

    log_z = xhat[2];
    log_ez = e[2];

    float u[4];

    float ex = xhat[0] - r[0];
    float ey = xhat[1] - r[1];
    float ez = xhat[2] - r[2];

    ez = clampFloatLocal(ez, -0.25f, 0.25f);

    u[0] = T_hover_total - K[0][2] * ez - K[0][5] * xhat[5] + Ki_z * int_z;
    u[1] = -(
      K[1][1] * ey +
      K[1][4] * xhat[4] +
      K[1][6] * xhat[6] +
      K[1][9] * xhat[9] +
      Ki_y * int_y
    );
    u[2] = -(
      K[2][0] * ex +
      K[2][3] * xhat[3] +
      K[2][7] * xhat[7] +
      K[2][10] * xhat[10] +
      Ki_x * int_x
    );
    u[3] = -K[3][11] * xhat[11];

    u[0] = clampFloatLocal(u[0], 0.0f, 0.55f);

    log_thrust = u[0];
    log_torqueX = u[1];
    log_torqueY = u[2];
    log_torqueZ = u[3];

    control->thrustSi = u[0];
    control->torqueX = u[1];
    control->torqueY = u[2];
    control->torqueZ = u[3];
  }

  control->controlMode = controlModeForceTorque;
}

LOG_GROUP_START(PP)
LOG_ADD(LOG_FLOAT, z, &log_z)
LOG_ADD(LOG_FLOAT, ez, &log_ez)
LOG_ADD(LOG_FLOAT, thrust, &log_thrust)
LOG_ADD(LOG_FLOAT, tx, &log_torqueX)
LOG_ADD(LOG_FLOAT, ty, &log_torqueY)
LOG_ADD(LOG_FLOAT, tz, &log_torqueZ)
LOG_GROUP_STOP(PP)

PARAM_GROUP_START(PP)
PARAM_GROUP_STOP(PP)
