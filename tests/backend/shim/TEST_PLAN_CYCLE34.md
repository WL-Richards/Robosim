# HAL shim core - cycle 34 test plan (HAL_GetOutputsEnabled)

Status: implemented

Cycle 34 implements:

```cpp
HAL_Bool HAL_GetOutputsEnabled(void);
```

Primary WPILib references:

- `hal/DriverStation.h` release docs: `HAL_GetOutputsEnabled(void)` returns
  true if outputs are enabled.
- WPILib sim `DriverStation.cpp`: returns
  `newestControlWord.enabled && newestControlWord.dsAttached`.

## Decisions

- **D-C34-NO-SHIM:** No installed shim returns false.
- **D-C34-EMPTY-CACHE:** Installed shim with no cached `ds_state` returns
  false.
- **D-C34-ENABLED-AND-DS:** Cached DS state returns true only when both
  `kControlEnabled` and `kControlDsAttached` are set.
- **D-C34-ESTOP:** E-stop alone does not make outputs enabled. If enabled and
  dsAttached bits are both set, the v0 result is still true because this
  surface mirrors WPILib's boolean expression; separate robot-disable logic is
  owned by the control word source.
- **D-C34-LATEST-WINS:** The result observes the same latest DS cache updated
  by direct `poll()` and `HAL_RefreshDSData()`.

## Proposed tests

### C34-1 - no shim returns false

- **Layer / contract:** Layer 2 C HAL no-shim behavior.
- **Bug class caught:** null global shim dereference or defaulting to enabled.
- **Inputs:** no installed shim; call `HAL_GetOutputsEnabled()`.
- **Expected:** returns `0`.

### C34-2 - empty DS cache returns false

- **Layer / contract:** Layer 2 empty-cache behavior.
- **Bug class caught:** fabricating enabled state before any DS packet.
- **Inputs:** installed connected shim with no `ds_state`.
- **Expected:** returns `0`.

### C34-3 - outputs are enabled only when enabled and DS attached bits are set

- **Layer / contract:** WPILib `newestControlWord.enabled &&
  newestControlWord.dsAttached` contract mapped to `ds_state::control`.
- **Bug class caught:** using enabled alone, dsAttached alone, or any nonzero
  control word as true.
- **Inputs:** cached DS states with control bits: none, enabled only,
  dsAttached only, enabled|dsAttached.
- **Expected:** only enabled|dsAttached returns `1`; all others return `0`.

### C34-4 - e-stop bit does not override the pinned boolean expression

- **Layer / contract:** exact WPILib sim expression rather than inferred safety
  policy.
- **Bug class caught:** treating eStop alone as enabled or adding hidden
  `!eStop` gating not present in the source expression.
- **Inputs:** cached DS states with `kControlEStop` only and with
  `kControlEnabled | kControlDsAttached | kControlEStop`.
- **Expected:** eStop-only returns `0`; enabled|dsAttached|eStop returns `1`.

### C34-5 - latest refresh update controls outputs-enabled result

- **Layer / contract:** `HAL_GetOutputsEnabled` observes the latest DS cache
  populated by `HAL_RefreshDSData`.
- **Bug class caught:** stale output-enabled value or reading a cache separate
  from DS refresh/getters.
- **Inputs:** refresh first DS state with enabled|dsAttached; read true; refresh
  second DS state with dsAttached only; read false.
- **Expected:** first read `1`; second read `0`.

## Deferred

- User-program observer calls are Cycle 35.
- Joystick outputs/rumble remain future work.
