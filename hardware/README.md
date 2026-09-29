# Hardware validation

The hardware portion of this project modified the Crazyflie firmware so the custom controller could send **force/torque commands** into a custom actuator-allocation path.

## Source provenance

The uploaded course firmware archive is now preserved in a minimal form under `upstream-template/`. Inspection shows that archive contains the starter firmware with TODO hooks rather than the final controller implementation. The project-specific files in `firmware/` are therefore still the versions reconstructed from the complete code listings embedded in the submitted final report.

They are intentionally presented as **project-specific reference code**, not as a complete standalone Crazyflie firmware fork.

### `controller_pp.c`

The report preserves the original course-template attribution to **Will Wu (2025)**. The project-specific work fills in the controller logic used for the Crazyflie hardware tests:

- 12-state feedback;
- altitude / lateral / forward / yaw channels;
- altitude reference clamping;
- takeoff-dependent x/y integral logic;
- force/torque output through `controlModeForceTorque`;
- logging for altitude, error, thrust, and body torques.

The version committed here preserves the logic shown in the final report, including the recorded integral resets after clamping. That behavior is documented rather than silently "fixed".

### `power_distribution_force_torque.c`

The submitted firmware was based on Bitcraze's stock `power_distribution_quadrotor.c`, which is GPL-3.0 licensed. To avoid republishing the entire upstream source file, this repository includes only the custom force/torque allocation portion:

- wrench-to-motor-thrust inversion;
- feasibility checking;
- common moment scaling by `alpha`;
- quadratic thrust-to-PWM conversion;
- motor PWM and desired-wrench logging.

To reproduce the original firmware experiment, merge this logic into a compatible Crazyflie firmware tree and retain the upstream Bitcraze license/header.

## Hardware result

The first custom-controller flight flipped immediately after takeoff. Logged allocation scale values indicated that the requested moments were too aggressive, which motivated larger LQR control penalties. Additional debugging checked Crazyflie sign/body-frame conventions and delayed x/y integral accumulation until altitude exceeded 0.15 m.

The final hardware test achieved more reasonable behavior and partial lift-off, but not a stable 0.5 m hover. The repository keeps that simulation-to-hardware gap explicit.

## License note

The firmware-derived allocation code is accompanied by the upstream Bitcraze GPL-3.0 license in `hardware/firmware/LICENSE-GPL-3.0.txt`.


## Uploaded starter firmware

The relevant source files from the uploaded firmware archive are kept under `upstream-template/`.
This makes it explicit what the project started from and where the custom controller/mixer logic was added,
without vendoring the entire Crazyflie firmware repository.
