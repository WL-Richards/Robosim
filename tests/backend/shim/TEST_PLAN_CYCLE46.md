# HAL shim core - cycle 46 test plan (HAL_GetComments)

Status: implemented

Cycle 46 adds the HALBase comments metadata query:

```cpp
void HAL_GetComments(struct WPI_String* comments);
```

WPILib's roboRIO implementation reads the comments string from
`/etc/machine-info` by parsing `PRETTY_HOSTNAME="..."`, unescapes the C-style
quoted contents, caches the parsed value, caps it to 64 bytes, and copies it
into a caller-owned `WPI_String`. The shim keeps this as a local HALBase
metadata surface: no protocol schema, no boot descriptor field, and no sim-core
host publication.

## Decisions

- **D-C46-C-ABI-ONLY:** Cycle 46 adds only `HAL_GetComments`. It does not
  change protocol schemas, boot metadata, or `shim_core` state.
- **D-C46-FILE-BACKED:** v0 reads comments from `/etc/machine-info` using the
  WPILib key `PRETTY_HOSTNAME="..."`. This matches the roboRIO source instead
  of inventing a simulator-only environment variable.
- **D-C46-TEST-PATH-OVERRIDE:** Production uses `/etc/machine-info`, but tests
  may set a shim-local machine-info path override and reset the comments cache.
  The override is not a HAL ABI symbol and exists only in the C++ shim namespace
  so tests never depend on or mutate the developer machine's real `/etc`. Each
  test using the override resets the cache before its first comments call and
  restores/clears the override afterward; C46-8 is the only test that
  intentionally avoids resetting between two calls.
- **D-C46-EMPTY-DEFAULT:** If the machine-info file cannot be opened, the key is
  absent, the value is empty, the quoted value is malformed/unterminated, or
  parsing yields no bytes, the result is an empty `WPI_String` (`{nullptr, 0}`
  in this shim's existing string helper convention).
- **D-C46-UNESCAPE-SUBSET:** v0 supports the escape forms needed for normal
  machine-info comments: `\"`, `\\`, `\n`, `\t`, and `\r`. Unknown escapes
  preserve the escaped character. The slice does not implement octal/hex/unicode
  escape decoding.
- **D-C46-WPILIB-64-BYTE-CAP:** WPILib stores the cached comments bytes in a
  fixed `char[64]`; the shim returns at most the first 64 unescaped bytes.
- **D-C46-CACHE:** The first call initializes a process-local comments cache.
  Later calls return the cached value even if the file changes, matching
  WPILib's one-time initialization behavior.
- **D-C46-NO-SHIM-REQUIRED:** Like `HAL_GetSerialNumber`, comments metadata is
  process/host backed and does not require an installed shim.
- **D-C46-NO-PUBLISH-NO-POLL:** `HAL_GetComments` must not publish outbound
  messages, poll inbound messages, or mutate sim state.
- **D-C46-NULL-OUTPUT:** Passing null for `comments` is undefined behavior,
  matching the existing string-returning HAL shim functions.

## Proposed tests

### C46-1 - signature matches WPILib
- Layer / contract: C HAL ABI.
- Bug class caught: wrong return type or out-parameter type.
- Inputs: compile-time function pointer check.
- Expected: `HAL_GetComments` has type `void (*)(WPI_String*)`.
- Tolerance / determinism: compile-time exact.

### C46-2 - missing machine-info returns empty without shim
- Layer / contract: D-C46-EMPTY-DEFAULT and D-C46-NO-SHIM-REQUIRED.
- Bug class caught: requiring an installed shim, leaking host fallback text, or
  returning a nonempty default.
- Inputs: clear global shim; force the machine-info path to a missing file; call
  `HAL_GetComments`.
- Expected: empty `WPI_String`.
- Tolerance / determinism: exact bytes.

### C46-3 - absent PRETTY_HOSTNAME returns empty
- Layer / contract: parser key selection.
- Bug class caught: reading the wrong machine-info field or returning the whole
  file.
- Inputs: force machine-info path to a temp file containing other keys.
- Expected: empty `WPI_String`.
- Tolerance / determinism: exact bytes.

### C46-4 - malformed PRETTY_HOSTNAME returns empty
- Layer / contract: D-C46-EMPTY-DEFAULT malformed-value behavior.
- Bug class caught: reading past malformed input, returning partial unterminated
  text, or using unchecked parser state.
- Inputs: force machine-info path to a temp file containing
  `PRETTY_HOSTNAME="unterminated`.
- Expected: empty `WPI_String`.
- Tolerance / determinism: exact bytes.

### C46-5 - quoted PRETTY_HOSTNAME is copied exactly
- Layer / contract: D-C46-FILE-BACKED.
- Bug class caught: including quotes, truncating ordinary strings, or reading
  only until whitespace.
- Inputs: temp machine-info with `PRETTY_HOSTNAME="Practice Robot 7"`.
- Expected: `Practice Robot 7`.
- Tolerance / determinism: exact bytes.

### C46-6 - supported C-style escapes are unescaped
- Layer / contract: D-C46-UNESCAPE-SUBSET.
- Bug class caught: returning raw escape backslashes or mishandling embedded
  quote/backslash/newline/tab/carriage-return escapes.
- Inputs: temp machine-info with escaped quote, backslash, newline, tab, and
  carriage return.
- Expected: returned bytes contain the unescaped characters.
- Tolerance / determinism: exact bytes.

### C46-7 - comments are capped to 64 unescaped bytes
- Layer / contract: D-C46-WPILIB-64-BYTE-CAP.
- Bug class caught: returning arbitrarily long comments, capping raw escaped
  bytes before unescaping, or off-by-one truncation.
- Inputs: temp machine-info with a `PRETTY_HOSTNAME` value longer than 64 bytes.
- Expected: returned string is exactly the first 64 unescaped bytes.
- Tolerance / determinism: exact byte count and contents.

### C46-8 - first call caches comments
- Layer / contract: D-C46-CACHE.
- Bug class caught: re-reading comments on each call and diverging from WPILib's
  one-time initialization behavior.
- Inputs: temp machine-info initially contains one comments value; call
  `HAL_GetComments`; rewrite file with a different value; call again without
  resetting the comments cache.
- Expected: both calls return the first value.
- Tolerance / determinism: exact bytes.

### C46-9 - installed shim does not publish or poll
- Layer / contract: D-C46-NO-PUBLISH-NO-POLL.
- Bug class caught: shortcut behavior where a metadata query sends protocol
  messages or consumes pending inbound state.
- Inputs: installed shim with a pending inbound `clock_state` in the
  core-to-backend lane; forced machine-info temp file contains a comments value.
- Expected: comments value is returned; backend-to-core lane remains empty;
  core-to-backend lane remains full; clock/power/DS caches remain unset.
- Tolerance / determinism: exact values, lane state, and cache emptiness.

### C46-10 - remains available after shutdown detach
- Layer / contract: D-C46-NO-SHIM-REQUIRED.
- Bug class caught: accidentally tying comments metadata to the installed shim
  pointer.
- Inputs: connected installed shim; call `HAL_Shutdown`; call
  `HAL_GetComments`.
- Expected: comments value is still returned after global shim detach.
- Tolerance / determinism: exact bytes and null current shim.

## Deferred

- Hardware-profile metadata source for comments.
- Full `wpi::UnescapeCString` parity for octal/hex/unicode escapes.
- Exposing a production/public reset hook for the comments cache.
