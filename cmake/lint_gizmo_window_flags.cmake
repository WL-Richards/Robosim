# lint_gizmo_window_flags.cmake — TEST_PLAN_VL.md G1.
#
# Pins that the Gizmo controls window in src/viz/main.cpp does not
# carry ImGuiWindowFlags_NoSavedSettings — that flag would drop the
# Gizmo window's position from the persisted layout.
#
# Note: ImGuiWindowFlags_NoDocking is intentional on the Gizmo
# (it floats as a transparent HUD; docked windows render the dock
# node's background instead of WindowBg, defeating the transparency).
# Only NoSavedSettings is banned.
#
# Approach: locate the body of `void draw_gizmo_controls(` in
# src/viz/main.cpp by brace-matching from the function header to its
# closing brace at column 1, then assert neither banned flag appears
# inside that span. Same posture as scripts/lint.sh's banned-clock
# check and cmake/lint_renderer_isolation.cmake — the build itself
# refuses to produce a Gizmo window that can't dock or persist.
#
# Known limitation: text-only grep. A `using` alias or a constexpr
# rebinding the flag could slip past it. The codebase doesn't use
# such aliases and a comment in main.cpp documents the constraint.

function(robosim_viz_check_gizmo_window_flags)
  set(_main_cpp "${CMAKE_SOURCE_DIR}/src/viz/main.cpp")
  if(NOT EXISTS "${_main_cpp}")
    message(FATAL_ERROR
      "G1 gizmo-window-flags: ${_main_cpp} not found.")
  endif()

  file(READ "${_main_cpp}" _content)

  # Locate the function header.
  string(FIND "${_content}" "void draw_gizmo_controls(" _start)
  if(_start EQUAL -1)
    message(FATAL_ERROR
      "G1 gizmo-window-flags: 'void draw_gizmo_controls(' not found in "
      "${_main_cpp}. If the function was renamed or moved, update "
      "cmake/lint_gizmo_window_flags.cmake to match.")
  endif()

  # Find the function body's closing brace at column 1 — i.e. a
  # newline followed by a single '}'.
  string(SUBSTRING "${_content}" ${_start} -1 _from_func)
  string(FIND "${_from_func}" "\n}" _end_rel)
  if(_end_rel EQUAL -1)
    message(FATAL_ERROR
      "G1 gizmo-window-flags: could not locate end of "
      "draw_gizmo_controls in ${_main_cpp}.")
  endif()

  string(SUBSTRING "${_from_func}" 0 ${_end_rel} _body)

  set(_banned_flags
    "ImGuiWindowFlags_NoSavedSettings"
  )
  foreach(_flag IN LISTS _banned_flags)
    string(FIND "${_body}" "${_flag}" _hit)
    if(NOT _hit EQUAL -1)
      message(FATAL_ERROR
        "G1 gizmo-window-flags: 'draw_gizmo_controls' in ${_main_cpp} "
        "carries the banned flag '${_flag}'. The Gizmo window must be "
        "dockable and have its position saved so the layout persists. "
        "Drop the flag or — if it is genuinely required — update "
        "tests/viz/TEST_PLAN_VL.md and this lint together.")
    endif()
  endforeach()
endfunction()
