// Build-only translation unit per TEST_PLAN A1. #includes every payload
// header so each header's static_asserts (POD discipline, sizeof,
// alignof, padding hygiene) get checked at build time. If any
// static_assert fails, the schema-cycle library fails to compile and
// no test below ever runs — the failure is loud, immediate, and
// localized to the offending header.
//
// This file deliberately contains no GTest TEST() bodies. It exists so
// the headers are reached by the test target's compile, even if no
// other test source touches them.

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "joystick_output.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_version.h"
#include "sync_envelope.h"
#include "truncate.h"
#include "types.h"
#include "user_program_observer.h"
#include "validator.h"
#include "validator_error.h"
