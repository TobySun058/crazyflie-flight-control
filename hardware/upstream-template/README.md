# Uploaded firmware template

This directory preserves the relevant files extracted from the uploaded course firmware archive
`ese-4481-final-project-toby-josh-master.zip`.

The uploaded archive contains the **starter/template firmware**: both the pole-placement controller
and force/torque mixer locations are still marked with TODOs. The final project-specific controller
and allocation logic is therefore kept separately under `../firmware/`, where it was reconstructed
from the complete code listings in the submitted final report.

Files preserved here make the modification boundary auditable:

- `controller_pp.c` — starter controller hook
- `controller_pp.h` — controller interface
- `power_distribution_quadrotor.c` — stock Bitcraze distribution file with the course TODO hook

Archive identifier embedded in the uploaded ZIP: `d25f8f729d613050fcf559d16a995908d464b280`.

The Bitcraze-derived files are GPL-3.0 licensed. See `../firmware/LICENSE-GPL-3.0.txt`.
