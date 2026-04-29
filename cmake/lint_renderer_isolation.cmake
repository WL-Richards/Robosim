# lint_renderer_isolation.cmake — TEST_PLAN.md T1.
#
# Pins the architectural invariant that the visualizer renderer reads
# only from scene_snapshot, not from the robot_description loader.
# Bypassing this seam would couple the renderer to the loader's struct
# and make Live / Replay modes expensive to retrofit.
#
# Two checks at configure time:
#
# 1. Source grep — `src/viz/` files (with an explicit allow-list) must
#    not include any description/* header or reference the
#    `robosim::description` namespace. This is text-only; `using`
#    aliases or `auto`-deduced uses can sneak past it. v0 codebase
#    does not use such aliases.
#
# 2. Target-level link walk — robosim_viz_renderer's transitive link
#    closure must not contain robosim_description. Implemented via
#    INTERFACE_LINK_LIBRARIES walked to a fixed point.
#
# A failure here fails CMake configuration, not test execution. Same
# posture as scripts/lint.sh's banned-clock check: the build itself
# refuses to produce a renderer coupled to the loader.

function(robosim_viz_check_renderer_isolation)
  set(_viz_dir "${CMAKE_SOURCE_DIR}/src/viz")
  set(_allow_list
    "${_viz_dir}/edit_mode_apply.cpp"
    "${_viz_dir}/edit_mode_apply.h"
    "${_viz_dir}/edit_mode_builder.cpp"
    "${_viz_dir}/edit_mode_builder.h"
    "${_viz_dir}/edit_session.cpp"
    "${_viz_dir}/edit_session.h"
    "${_viz_dir}/panels.cpp"
    "${_viz_dir}/panels.h"
    "${_viz_dir}/main.cpp"
  )

  file(GLOB_RECURSE _viz_sources
    LIST_DIRECTORIES false
    "${_viz_dir}/*.cpp"
    "${_viz_dir}/*.h"
  )

  set(_forbidden_patterns
    "#include[ \t]*\"description/"
    "#include[ \t]*<description/"
    "robosim::description"
  )

  foreach(_src IN LISTS _viz_sources)
    list(FIND _allow_list "${_src}" _allow_idx)
    if(NOT _allow_idx EQUAL -1)
      continue()
    endif()
    file(READ "${_src}" _content)
    foreach(_pat IN LISTS _forbidden_patterns)
      string(REGEX MATCH "${_pat}" _match "${_content}")
      if(_match)
        message(FATAL_ERROR
          "T1 renderer-isolation: file '${_src}' references the "
          "description loader (pattern: '${_pat}'). The renderer reads "
          "only from scene_snapshot. If a new file legitimately needs "
          "the description namespace, add it to the allow-list in "
          "cmake/lint_renderer_isolation.cmake and document why.")
      endif()
    endforeach()
  endforeach()

  # Target-level walk: robosim_viz_renderer must not transitively link
  # robosim_description. The target lands in Phase VB5; until then this
  # half no-ops to keep VB1–VB4 from passing-by-skip.
  if(TARGET robosim_viz_renderer)
    set(_visited "")
    set(_pending "robosim_viz_renderer")
    while(_pending)
      list(POP_FRONT _pending _current)
      list(FIND _visited "${_current}" _seen_idx)
      if(NOT _seen_idx EQUAL -1)
        continue()
      endif()
      list(APPEND _visited "${_current}")
      if(NOT TARGET "${_current}")
        continue()
      endif()
      get_target_property(_iface "${_current}" INTERFACE_LINK_LIBRARIES)
      if(_iface)
        foreach(_dep IN LISTS _iface)
          if(_dep STREQUAL "robosim_description")
            message(FATAL_ERROR
              "T1 renderer-isolation: robosim_viz_renderer transitively "
              "links robosim_description (via '${_current}'). The "
              "renderer must read only from scene_snapshot.")
          endif()
          list(APPEND _pending "${_dep}")
        endforeach()
      endif()
      get_target_property(_priv "${_current}" LINK_LIBRARIES)
      if(_priv)
        foreach(_dep IN LISTS _priv)
          if(_dep STREQUAL "robosim_description")
            message(FATAL_ERROR
              "T1 renderer-isolation: robosim_viz_renderer transitively "
              "links robosim_description (via '${_current}'). The "
              "renderer must read only from scene_snapshot.")
          endif()
          list(APPEND _pending "${_dep}")
        endforeach()
      endif()
    endwhile()
  endif()
endfunction()
