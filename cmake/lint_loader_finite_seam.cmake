# lint_loader_finite_seam.cmake — TEST_PLAN_VC.md D-VC-3.
#
# Pins the architectural invariant that the loader's only
# non-finite-rejection mechanism is the per-scalar validator helper
# in src/description/origin_pose.cpp (validate_finite_xyz /
# validate_finite_rpy). A bare std::isfinite / std::isnan /
# std::isinf / std::isnormal / std::fpclassify call elsewhere in
# src/description/ would give the loader a parallel inline check
# that no test exercises — exactly the bug class
# `code-style.md`'s "no shortcuts" non-negotiable forbids.
#
# Posture: configure-time tripwire against ACCIDENTAL drift, backed
# by V9 (subnormals load successfully) at runtime. NOT closure-tight
# against deliberate evasion (bit-cast finite-checks, third-TU
# wrapper functions). Deliberate evasion is handled culturally by
# code-style.md, not by this lint. A future maintainer should not
# extend the tokenset to chase every plausible workaround.
#
# A failure here fails CMake configuration, not test execution.
# Same posture as scripts/lint.sh's banned-clock check.

function(robosim_check_loader_finite_seam)
  set(_dir "${CMAKE_SOURCE_DIR}/src/description")
  # Helper TU is the only file allowed to call the banned tokens.
  # Update this list (and origin_pose.cpp's source comment) if the
  # helpers move to another TU.
  set(_allow_list
    "${_dir}/origin_pose.cpp"
  )

  set(_banned_tokens
    "isfinite"
    "isnan"
    "isinf"
    "isnormal"
    "fpclassify"
  )

  file(GLOB_RECURSE _sources
    LIST_DIRECTORIES false
    "${_dir}/*.cpp"
    "${_dir}/*.h"
  )

  foreach(_src IN LISTS _sources)
    list(FIND _allow_list "${_src}" _allow_idx)
    if(NOT _allow_idx EQUAL -1)
      continue()
    endif()
    file(READ "${_src}" _content)
    foreach(_tok IN LISTS _banned_tokens)
      # Match an actual function call: the token (preceded by either
      # start-of-string or a non-identifier char to keep `frexp` etc.
      # from tripping on substrings) followed immediately by `(`. We
      # match calls only — bare token mentions in comments / strings
      # don't fire, so a documentation block can list these names
      # without tripping the lint.
      string(REGEX MATCH "(^|[^A-Za-z_])${_tok}\\(" _match "${_content}")
      if(_match)
        message(FATAL_ERROR
          "D-VC-3 loader-finite-seam: file '${_src}' contains a "
          "bare '${_tok}' call. The loader must call "
          "validate_finite_xyz / validate_finite_rpy in "
          "origin_pose.cpp as its sole non-finite-rejection "
          "mechanism. See tests/description/TEST_PLAN_VC.md D-VC-3.")
      endif()
    endforeach()
  endforeach()
endfunction()
