# src/ — sim core C++ tree.
#
# Subdirectories mirror the layer model in docs/ARCHITECTURE.md:
#   - backend/   — Layer 2: HAL shim, state cache, sync (per-tier transport)
#   - bus/       — Layer 3: device bus arbitration model
#   - vendors/   — Layer 3: vendor firmware models
#   - models/    — Layer 4: physical models (motors, sensors, electrical)
#   - world/     — Layer 5: dynamics interface + engine bindings
#   - description/ — robot description loader
#   - sim_core/  — process entry point
#   - time/, rng/, log/, util/ — shared infrastructure

add_subdirectory(util)
add_subdirectory(time)
add_subdirectory(rng)
add_subdirectory(log)
add_subdirectory(description)
add_subdirectory(backend)
add_subdirectory(bus)
add_subdirectory(vendors)
add_subdirectory(models)
add_subdirectory(world)
add_subdirectory(sim_core)
add_subdirectory(viz)
