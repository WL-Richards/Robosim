# tests/ — mirrors src/ exactly. GoogleTest + GoogleMock.
#
# A test binary per leaf directory keeps test scope obvious and keeps
# link times reasonable as the suite grows. Use add_robosim_test() (a
# tiny helper added when the first real test lands) to avoid repeating
# target_link_libraries boilerplate.

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
