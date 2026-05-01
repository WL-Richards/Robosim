# Visualizer layout persistence (Phase VL) — test plan

**Status:** **rev 4, awaiting re-review.** Implements a small UX
follow-up under `docs/VISUALIZER_V0_PLAN.md` "Open follow-ups": panel
layout (Scene tree, Inspector, Gizmo controls) is currently undocked at
launch and the saved `imgui.ini` lives in the user's CWD (so launching
from a different directory loses it). This phase:

1. Persists ImGui state to a stable per-user path
   (`$XDG_CONFIG_HOME/robosim-viz/imgui.ini`, falling back to
   `$HOME/.config/robosim-viz/imgui.ini`).
2. Builds a default dock layout on first launch (no ini at the resolved
   path):
   - **Scene** docked top-right.
   - **Inspector** docked below Scene (also right column).
   - **Gizmo** docked top-left.
   - Central node is the 3D viewport (passthrough).
3. Lets the **Gizmo** window dock and persist (today it carries
   `NoDocking | NoSavedSettings`; both are dropped).

Once the user rearranges, the rearranged state writes back to
`imgui.ini` on shutdown via ImGui's stock auto-save path; the default
layout only fires when the file is absent.

**Review history:**
- rev 1 → `not-ready`. Three correctable findings:
  - **L4** degenerate against an implementation that drops the XDG
    branch entirely. Resolved in rev 2 by adding a second parameterized
    case (`relative XDG + null HOME → nullopt`).
  - **L5–L10** referenced `testing::TempDir()` in plain `TEST()`,
    which doesn't compile that way. Resolved by switching the temp-dir
    tests to a `TEST_F` fixture (`LayoutFsTest`) with `SetUp`/
    `TearDown` for hermetic per-test scratch directories.
  - **D1** used `FindWindowByName(...)->DockId`, which returns null
    until a window is `Begin`-submitted. Rev 2 attempted to fix this
    by asserting on the dock-node `Windows` ImVector — which the
    rev-2 reviewer correctly flagged as a different broken API
    assumption (see rev-3 fix below).
- rev 2 → `not-ready`. Rev-2 reviewer findings:
  - **D1-A (crash):** `ImGui::GetID("test_dockspace")` dereferences
    `GImGui->CurrentWindow`, which is null before `NewFrame()`.
    Rev-3 fix: use `ImHashStr("test_dockspace")` directly (declared
    `IMGUI_API` in `imgui_internal.h`), or capture the ID via
    `DockBuilderAddNode(0, ImGuiDockNodeFlags_DockSpace)`.
  - **D1-B (wrong observable):** `DockBuilderDockWindow(name, id)`
    *without* an existing `ImGuiWindow*` writes only to
    `ImGuiWindowSettings::DockId`; `DockBuilderFinish` then iterates
    `g.Windows` (the submitted-window list, empty without `Begin`
    calls), so each leaf node's `Windows` ImVector remains empty.
    Cited from imgui.cpp lines 20698–20721 and 21099–21104. Rev-3
    fix: replace the `node->Windows` walk with
    `FindWindowSettingsByID(ImHashStr(name))->DockId` and compare
    against the expected leaf node IDs returned by the implementation
    via `DockBuilderSplitNode`'s out-params.
  - All other rev-2 changes (L4 case B, `LayoutFsTest`, G1
    configure-time lint, suite-level cleanup) were approved
    unchanged.
- rev 3 fixes both D1-A and D1-B; no other test changes.
- rev 3 → `not-ready` (approve-with-changes) on a single load-
  bearing implementation constraint that was implicit but not
  stated: `apply_default_dock_layout`'s call to `DockBuilderAddNode`
  must NOT pass `ImGuiDockNodeFlags_DockSpace` — that flag dispatches
  to `ImGui::DockSpace()` which dereferences `g.CurrentWindow` and
  crashes without an active frame (imgui.cpp lines 20767–20770).
  Rev-4 fix: documented the constraint in both the
  `apply_default_dock_layout` doc comment and in D1 step 4. The
  production caller in `main.cpp` already creates the dockspace via
  `ImGui::DockSpaceOverViewport(...)` each frame, so the helper is
  only setting up the *layout* of an already-alive dockspace, never
  creating one — the `0` flags path matches both production and test
  needs.

**Implementer's first action:** confirm rev-2 verdict from
`test-reviewer`. Iterate to `ready-to-implement` before writing test
code.

---

## Determinism note (visible to test-reviewer)

The visualizer is exempt from sim-core determinism rules
(`.claude/skills/visualizer.md` "Determinism exception"). Tests under
`tests/viz/` may touch the filesystem; no wall-clock or RNG is used in
this plan. Tests remain deterministic in the sense that fits this
project's contract (same env / FS state → same output).

The L1–L4 tests pass `const char*` arguments to
`resolve_imgui_ini_path` directly; no `setenv`/`unsetenv` is needed
because the public surface takes the env-var values as arguments
rather than reading `getenv` internally. The single caller in
`main.cpp` reads `getenv` once and passes the results in.

---

## Public surface under test

```cpp
namespace robosim::viz {

// Per-user persistent path for ImGui's saved state (positions, dock
// layout). Resolution order:
//   1. $XDG_CONFIG_HOME/robosim-viz/imgui.ini (if XDG_CONFIG_HOME is
//      a non-empty absolute path).
//   2. $HOME/.config/robosim-viz/imgui.ini   (if HOME is non-empty).
//   3. std::nullopt — caller falls back to ImGui's default ("imgui.ini"
//      in CWD).
//
// Argument-driven (does not call getenv); the caller in main.cpp
// reads the env vars once and forwards them.
[[nodiscard]] std::optional<std::filesystem::path> resolve_imgui_ini_path(
    const char* xdg_config_home,
    const char* home);

// Ensures the parent directory of `ini_path` exists. Returns true on
// success (directory existed or was created), false on filesystem
// error. Implementation is a thin wrapper over
// std::filesystem::create_directories with error_code; the test value
// is pinning the contract that callers don't have to handle the
// "directory missing" case before ImGui writes its ini.
[[nodiscard]] bool ensure_ini_parent_dir(
    const std::filesystem::path& ini_path);

// True iff `ini_path` does not currently exist on disk. Used by the
// caller to decide whether to apply the default dock layout (only on
// first run; subsequent runs let ImGui restore the saved layout).
//
// Caller's responsibility: passing a path that exists but is a
// directory (rather than a regular file) is malformed input. The
// predicate returns false in that case (the path "exists") and the
// caller will then not apply the default layout; ImGui's later
// attempt to write to that path will fail and the user will see the
// (separate) ini-write error path. Not separately tested.
[[nodiscard]] bool should_apply_default_layout(
    const std::filesystem::path& ini_path);

// Names of the windows the default layout docks. Pinned as constants
// so tests don't drift from the panels' window titles. Match the
// titles used in panels.cpp (`Scene`, `Inspector`) and main.cpp
// (`Gizmo`).
inline constexpr const char* default_layout_scene_window     = "Scene";
inline constexpr const char* default_layout_inspector_window = "Inspector";
inline constexpr const char* default_layout_gizmo_window     = "Gizmo";

// Out-param record exposing the leaf node IDs that
// `apply_default_dock_layout` produced. Tests compare each
// pre-assigned window's `ImGuiWindowSettings::DockId` against these
// IDs; production callers ignore it.
//
// Pre-`Begin` observability: `DockBuilderDockWindow(name, id)`
// without an existing `ImGuiWindow*` writes only to
// `ImGuiWindowSettings::DockId`, not to the dock node's `Windows`
// ImVector. So tests must observe the assignment via
// `FindWindowSettingsByID(ImHashStr(name))->DockId`, not by walking
// `node->Windows`. (See imgui.cpp DockBuilderDockWindow lines
// 20698–20721 and DockContextBuildAddWindowsToNodes lines
// 21099–21104.)
struct dock_layout_node_ids {
  unsigned int scene_leaf_id     = 0;
  unsigned int inspector_leaf_id = 0;
  unsigned int gizmo_leaf_id     = 0;
  unsigned int central_id        = 0;
};

// Builds the first-run dock layout into `dockspace_id` using the
// ImGui::DockBuilder* API. Must be called while an ImGui context
// exists; does not require a window-frame submission.
//
// The implementation calls DockBuilderRemoveNode / AddNode /
// SetNodeSize / SplitNode / DockWindow / Finish in sequence.
//
// **Constraint (load-bearing for testability):** the
// `DockBuilderAddNode(dockspace_id, ...)` call MUST pass `0` flags,
// not `ImGuiDockNodeFlags_DockSpace`. With the `DockSpace` flag,
// `DockBuilderAddNode` dispatches to `ImGui::DockSpace()`
// (imgui.cpp ~line 20767-20770), which calls
// `GetContentRegionAvail()` → `g.CurrentWindow`, crashing without
// an active frame. Production code does not need the flag because
// the dockspace node is already created by the caller's per-frame
// `ImGui::DockSpaceOverViewport(...)` call (see main.cpp), so the
// helper only configures an already-alive dockspace.
//
// If `out_node_ids` is non-null, the four leaf IDs (Scene,
// Inspector, Gizmo, central) are written so tests can verify
// per-window settings assignment. Production callers pass `nullptr`.
void apply_default_dock_layout(unsigned int dockspace_id,
                               dock_layout_node_ids* out_node_ids = nullptr);

}  // namespace robosim::viz
```

The header lives at `src/viz/layout.h`; implementation at
`src/viz/layout.cpp`. The new symbols link in via a new
`robosim_viz_layout` static library that depends on
`robosim_viz_imgui` (for `apply_default_dock_layout`) and the standard
library (for `resolve_imgui_ini_path` / `ensure_ini_parent_dir` /
`should_apply_default_layout`).

### Tolerance policy

All tests in this plan are exact (string / boolean / path equality);
no floating-point math is involved.

### Test fixtures

A single GoogleTest fixture covers all filesystem-touching tests:

```cpp
class LayoutFsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto* info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    // Use std::filesystem::temp_directory_path() (process temp dir,
    // typically /tmp) plus suite + test names so per-test scratch
    // dirs cannot collide across parallel runs.
    scratch_ = std::filesystem::temp_directory_path() /
               (std::string("robosim_viz_layout_") +
                info->test_suite_name() + "_" + info->name());
    std::filesystem::remove_all(scratch_);
    std::filesystem::create_directories(scratch_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(scratch_, ec);  // tolerate races
  }

  std::filesystem::path scratch_;
};
```

`TEST_F(LayoutFsTest, ...)` is required for any test that creates or
inspects files; plain `TEST(...)` is used only for the pure-arg L1–L4
cases.

For D1 (the ImGui-context test) we use a separate fixture:

```cpp
class DockLayoutTest : public ::testing::Test {
 protected:
  void SetUp() override {
    IMGUI_CHECKVERSION();
    ctx_ = ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1280.0F, 800.0F);
    io.IniFilename = nullptr;  // suppress disk I/O
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.Fonts->Build();
  }

  void TearDown() override {
    ImGui::DestroyContext(ctx_);
  }

  ImGuiContext* ctx_ = nullptr;
};
```

---

## L. Layout module — pure-logic tests

### L1. `resolve_imgui_ini_path_uses_xdg_config_home_when_set`

- Layer / contract: `resolve_imgui_ini_path` honors `$XDG_CONFIG_HOME`
  per the XDG Base Directory spec.
- Bug class: `$HOME` shadowing `$XDG_CONFIG_HOME` (precedence inverted),
  or a hard-coded `~/.config` ignoring the env var.
- Procedure:
  ```cpp
  auto p = resolve_imgui_ini_path("/tmp/xdg-fake", "/home/anybody");
  ```
- Expected:
  - `p.has_value() == true`.
  - `*p == std::filesystem::path("/tmp/xdg-fake/robosim-viz/imgui.ini")`.
- Tolerance: exact path equality.
- `TEST(LayoutPath, ...)`.

### L2. `resolve_imgui_ini_path_falls_back_to_home_dot_config_when_xdg_unset`

- Layer / contract: when `$XDG_CONFIG_HOME` is null or empty, fall back
  to `$HOME/.config`.
- Bug class: empty-string `$XDG_CONFIG_HOME` interpreted as a valid
  base (resolves to `/robosim-viz/imgui.ini`), or null check missing.
- Parameterized over two equivalent "unset" representations:
  - `xdg_config_home == nullptr`.
  - `xdg_config_home == ""`.
- Procedure: `resolve_imgui_ini_path(xdg, "/home/will")`.
- Expected:
  - `p.has_value() == true`.
  - `*p == "/home/will/.config/robosim-viz/imgui.ini"`.
- `TEST_P(LayoutPathUnsetXdg, ...)` with
  `INSTANTIATE_TEST_SUITE_P(NullAndEmpty, ...)`.

### L3. `resolve_imgui_ini_path_returns_nullopt_when_neither_env_var_set`

- Layer / contract: with no env-var basis, signal "no opinion" so the
  caller leaves ImGui's default in effect (`imgui.ini` in CWD). This
  is the correct posture for sandboxes / CI without `$HOME`.
- Bug class: returning a relative path or an empty path that ImGui
  would interpret as "save to /imgui.ini" (root) or "disable saving."
- Parameterized over the four null/empty combinations:
  - `(nullptr, nullptr)`.
  - `(nullptr, "")`.
  - `("", nullptr)`.
  - `("", "")`.
- Expected: `resolve_imgui_ini_path(...) == std::nullopt`.

### L4. `resolve_imgui_ini_path_rejects_relative_xdg_config_home` (parameterized)

- Layer / contract: per XDG spec, `$XDG_CONFIG_HOME` must be an
  absolute path; relative values are ignored as malformed.
- Bug class:
  - **(case A)** A malformed env var poisons the resolved path so the
    ini ends up somewhere arbitrary relative to CWD. (caught when
    HOME is set: result must be HOME-derived, not relative-XDG-
    derived.)
  - **(case B)** An implementation that "always skips the XDG branch
    entirely" coincidentally satisfies case A. To pin the **rejection**
    rather than the **fall-through**, case B exercises relative-XDG
    with HOME unavailable; a correct implementation returns
    `std::nullopt` (no usable basis) — a "skip XDG, always read HOME"
    implementation also returns `nullopt` here, but **case A** then
    forces it to use HOME, and the conjunction of the two is what
    makes the contract real.
- Parameterized over two cases:
  - **A:** `resolve_imgui_ini_path("relative/path", "/home/will")` →
    `*p == "/home/will/.config/robosim-viz/imgui.ini"`.
  - **B:** `resolve_imgui_ini_path("relative/path", nullptr)` →
    `p == std::nullopt`.
- Tolerance: exact.
- Notes: this is the rev-2 fix for the rev-1 reviewer finding that
  case A alone was degenerate — case B forces the relative-XDG
  rejection branch to be visited as a real code path, since a
  reasonable buggy implementation that mishandles relative XDG (e.g.
  passes it through `std::filesystem::absolute(rel)`) produces a
  CWD-relative absolute path, which fails case B (would return a
  CWD-derived ini path, not `nullopt`).

### L5. `ensure_ini_parent_dir_creates_missing_intermediate_directories`

- Layer / contract: `ensure_ini_parent_dir` recursively creates the
  parent directory chain.
- Bug class: a single-level `mkdir` call that fails on first run when
  `~/.config` itself doesn't yet exist (rare but legal).
- Procedure:
  - Within the `LayoutFsTest` fixture, use the per-test `scratch_`
    directory.
  - Path: `scratch_ / "a" / "b" / "c" / "imgui.ini"` (parent chain
    `a/b/c` does not exist; only `scratch_` exists).
  - Call `ensure_ini_parent_dir(path)`.
- Expected:
  - Return value `== true`.
  - `std::filesystem::is_directory(scratch_ / "a" / "b" / "c") == true`.
- Cleanup: `LayoutFsTest::TearDown` removes `scratch_` recursively.
- `TEST_F(LayoutFsTest, ...)`.

### L6. `ensure_ini_parent_dir_succeeds_when_directory_already_exists`

- Boundary: idempotent on subsequent runs.
- Procedure (within `LayoutFsTest`):
  - Pre-create `scratch_ / "x"`.
  - Call `ensure_ini_parent_dir(scratch_ / "x" / "imgui.ini")`.
- Expected: returns `true`; `scratch_ / "x"` still exists.
- `TEST_F(LayoutFsTest, ...)`.

### L7. `ensure_ini_parent_dir_returns_false_when_parent_is_an_existing_file`

- Boundary: caller passed a path whose parent collides with an
  existing file (not a directory).
- Procedure (within `LayoutFsTest`):
  - Create regular file `scratch_ / "clash"` with any contents
    (`std::ofstream(scratch_ / "clash") << "x";`).
  - Call `ensure_ini_parent_dir(scratch_ / "clash" / "imgui.ini")`.
- Expected: returns `false`.
- Bug class: throwing `filesystem_error` from a path that is "user's
  computer is weird" is a poor failure mode for a viz launch; it
  should degrade gracefully to ImGui's default. The test asserts on
  `false` return rather than absence of exception (the `false` return
  implies no exception terminated the call; a separate "no exception"
  assertion would be a code-smell anti-pattern per
  `tdd-workflow.md`).
- `TEST_F(LayoutFsTest, ...)`.

### L8. `should_apply_default_layout_returns_true_when_ini_missing`

- Layer / contract: caller decides whether to call DockBuilder based
  on this predicate.
- Procedure: `scratch_ / "never-written.ini"` (path does not exist).
- Expected: `should_apply_default_layout(...) == true`.
- `TEST_F(LayoutFsTest, ...)`.

### L9. `should_apply_default_layout_returns_false_when_ini_present`

- Symmetric to L8: respects the user's saved layout on subsequent
  runs.
- Procedure: write any non-empty bytes to `scratch_ / "saved.ini"`
  (`std::ofstream(scratch_ / "saved.ini") << "[Window]";`), then
  query.
- Expected: `should_apply_default_layout(...) == false`.
- `TEST_F(LayoutFsTest, ...)`.

### L10. `should_apply_default_layout_returns_false_for_empty_ini`

- Boundary: an empty file at the path counts as "user has been here"
  for the purpose of layout. Touching the file (e.g. `> imgui.ini`)
  intentionally suppresses the default layout.
- Procedure: create empty file `scratch_ / "empty.ini"` (open and
  immediately close an `ofstream`), then query.
- Expected: `should_apply_default_layout(...) == false`.
- Notes: alternative would be "treat empty as missing." Picking
  "present" matches the principle of least surprise — if the user
  manually emptied the file, they don't want us re-overwriting.
- `TEST_F(LayoutFsTest, ...)`.

---

## D. Default dock layout — ImGui-context test

### D1. `apply_default_dock_layout_assigns_panel_settings_to_distinct_leaf_nodes`

- Layer / contract: after `apply_default_dock_layout(id, &out)`
  runs in an ImGui context, (1) the dock-node tree under `id` has
  the structural shape (root split, central node present), and
  (2) each named window's `ImGuiWindowSettings::DockId` (the
  pre-`Begin`, pre-frame observable) equals the corresponding leaf
  ID returned via `out_node_ids`, and (3) the four leaf IDs are
  pairwise distinct (so Scene / Inspector / Gizmo / central are
  truly separate dock nodes).
- Bug class:
  - A future refactor swaps the split orientation, accidentally
    puts Inspector in the central node, or renames a window without
    updating the `default_layout_*` constants.
  - An implementation that "calls DockBuilderSplitNode but forgets
    DockBuilderDockWindow" leaves the windows unassociated; this
    test catches it because each window's
    `FindWindowSettingsByID(ImHashStr(name))` would be null (no
    settings created for that name), or `settings->DockId == 0`.
  - An implementation that aliases two leaves (e.g. Inspector docked
    to the same leaf as Scene by mistake) is caught by the
    pairwise-distinct assertion.
- **Procedure (rev-3 corrected):**
  1. `IMGUI_CHECKVERSION(); ImGui::CreateContext();` (in
     `DockLayoutTest::SetUp`).
  2. Set `ImGuiIO::DisplaySize = {1280, 800}`,
     `IniFilename = nullptr`, `ConfigFlags |= DockingEnable`. **Do
     not** call `ImGui::NewFrame()` — `DockBuilder` operates on the
     global dock context and does not require an active frame.
     `Fonts->Build()` is also not required (no rendering).
  3. **Do not** call `ImGui::GetID("test_dockspace")`. That helper
     dereferences `GImGui->CurrentWindow`, which is null without a
     prior `NewFrame()` (imgui.cpp ~line 9938). Use `ImHashStr`
     instead, which is a pure hash (declared `IMGUI_API` in
     `imgui_internal.h`):
     ```cpp
     const ImGuiID dockspace_id = ImHashStr("test_dockspace");
     ```
  4. Call `apply_default_dock_layout(dockspace_id, &node_ids)`. The
     implementation must internally call
     `ImGui::DockBuilderFinish(dockspace_id)` as its last step.
     **Implementation constraint that the test depends on:** the
     helper must call `DockBuilderAddNode(dockspace_id, 0)` —
     specifically NOT `ImGuiDockNodeFlags_DockSpace`. With that flag,
     `DockBuilderAddNode` dispatches to `ImGui::DockSpace`, which
     reaches `g.CurrentWindow` and crashes in this no-frame test
     context. The production caller (main.cpp) creates the dockspace
     each frame via `ImGui::DockSpaceOverViewport(...)`, so the
     helper only configures an existing dockspace — the `0` flags
     path is correct for both production and test.
  5. Verify the structural shape:
     ```cpp
     ImGuiDockNode* root = ImGui::DockBuilderGetNode(dockspace_id);
     ASSERT_NE(root, nullptr);
     EXPECT_TRUE(root->IsSplitNode());

     ImGuiDockNode* central =
         ImGui::DockBuilderGetCentralNode(dockspace_id);
     ASSERT_NE(central, nullptr);
     EXPECT_TRUE(central->IsCentralNode());
     EXPECT_EQ(central->ID, node_ids.central_id);
     ```
  6. Verify each named window's settings point at the expected leaf
     (this is the pre-`Begin` observable, as established by the
     rev-2 reviewer):
     ```cpp
     auto dock_id_for = [](const char* name) -> ImGuiID {
       ImGuiWindowSettings* s =
           ImGui::FindWindowSettingsByID(ImHashStr(name));
       return s != nullptr ? s->DockId : 0;
     };
     EXPECT_EQ(dock_id_for(default_layout_scene_window),
               node_ids.scene_leaf_id);
     EXPECT_EQ(dock_id_for(default_layout_inspector_window),
               node_ids.inspector_leaf_id);
     EXPECT_EQ(dock_id_for(default_layout_gizmo_window),
               node_ids.gizmo_leaf_id);
     ```
  7. Verify the four leaf IDs are pairwise distinct (so Scene,
     Inspector, Gizmo, and the central node are truly separate dock
     nodes; pins the "Scene+Inspector are split, not tabbed" and the
     "Gizmo is its own node" decisions):
     ```cpp
     std::array<ImGuiID, 4> ids = {
         node_ids.scene_leaf_id, node_ids.inspector_leaf_id,
         node_ids.gizmo_leaf_id, node_ids.central_id};
     for (std::size_t i = 0; i < ids.size(); ++i) {
       EXPECT_NE(ids[i], 0u) << "leaf " << i << " was not produced";
       for (std::size_t j = i + 1; j < ids.size(); ++j) {
         EXPECT_NE(ids[i], ids[j])
             << "leaves " << i << " and " << j << " collided";
       }
     }
     ```
- Tolerance: exact (`ImGuiID` integer equality, pointer equality,
  boolean).
- **Decision pin (Scene + Inspector are vertically split, not
  tabbed):** the implementation calls
  `DockBuilderSplitNode(right_id, ImGuiDir_Up, 0.5F, &top_id,
  &bottom_id)` and docks Scene into `top_id` and Inspector into
  `bottom_id`. Tabs would put both windows in the same node,
  collapsing `node_ids.scene_leaf_id == node_ids.inspector_leaf_id`
  — the pairwise-distinct assertion in step 7 catches this.
- **Decision pin (sizing — UX, not behavior):** the right column
  takes 25% of the window width; the top-left Gizmo node takes 30%
  of the left column's height. Constants live in `layout.cpp` as
  named `constexpr` ratios; tests assert the layout *shape* (which
  window is bound to which leaf, that the four leaves are distinct),
  not the exact ratios. UX tuning of the ratios should not break
  this test.
- **ImGui-internals citations** (so a future implementer can verify
  the procedure against a different ImGui version):
  - `DockBuilderDockWindow` writes only to
    `ImGuiWindowSettings::DockId` when no `ImGuiWindow*` exists —
    imgui.cpp lines 20698–20721 (`else` branch).
  - `DockBuilderFinish` calls `DockContextBuildAddWindowsToNodes`,
    which iterates `g.Windows` (empty without `Begin` calls) —
    imgui.cpp lines 21099–21104.
  - `FindWindowSettingsByID` and `ImHashStr` are declared
    `IMGUI_API` in `imgui_internal.h` (lines ~3506 and ~381 in the
    pinned SHA), so test code that includes `imgui_internal.h` can
    call them directly.
- `TEST_F(DockLayoutTest, ...)`.

---

## G. Gizmo window flags — configure-time grep

### G1. `lint_gizmo_window_flags_disallow_no_docking` (CMake configure-time)

- **Not a GoogleTest test; a CMake configure-time structural check.**
  Lives at `cmake/lint_gizmo_window_flags.cmake`, invoked from the
  top-level `CMakeLists.txt` when `ROBOSIM_BUILD_VIZ=ON` (alongside
  the existing `lint_renderer_isolation.cmake` call).
- **What it does:** opens `src/viz/main.cpp`, locates the
  `draw_gizmo_controls` function body (a heredoc-style scan between
  `void draw_gizmo_controls(` and the next `}` at column 1), and
  asserts the substring `ImGuiWindowFlags_NoDocking` does NOT appear
  inside that span. Same for `ImGuiWindowFlags_NoSavedSettings`.
- Configure fails (not test-time fails) if either flag is found.
- **Bug class:** a future change re-adds `NoDocking` "to keep the
  Gizmo window floating," silently breaking the persisted layout
  contract. The configure-time gate makes the build itself refuse to
  produce a Gizmo window that can't dock or persist.
- **Why configure-time and not GoogleTest:** a runtime test cannot
  cheaply pin "this window does not have flag X" without standing up
  a full ImGui context, calling into `draw_gizmo_controls` at the
  right point in a frame, and reading its `ImGuiWindow::Flags` after
  submission. A grep is sufficient and matches the existing
  `lint_renderer_isolation.cmake` posture.
- **Known limitation:** text-only grep — a `using
  ImGuiWindowFlags = ...; constexpr auto BadFlag = NoDocking;` alias
  could sneak past it. v0 doesn't use such aliases; documented in
  the lint script.
- **Pre-rev-1 baseline:** this lint would currently FIRE because
  `main.cpp:425–426` reads:
  ```cpp
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking |
                           ImGuiWindowFlags_AlwaysAutoResize |
                           ImGuiWindowFlags_NoSavedSettings;
  ```
  The implementer must drop the two banned flags as part of this
  phase (leaving `AlwaysAutoResize` is fine — it's a sizing affordance,
  not a docking/persistence flag).

---

## Coverage I am explicitly NOT testing

- **End-to-end "ImGui actually saves to the resolved path."** That
  contract is owned by ImGui itself (`ImGuiIO::IniFilename` is read
  by `ImGui::SaveIniSettingsToDisk` on `ImGui::EndFrame`). The smoke
  test verifies this implicitly (set IniFilename, run a frame, exit;
  the file appears).
- **DockBuilder split ratios.** UX tuning, not behavior. If the test
  pinned ratios, every cosmetic adjustment would re-trigger this
  test. Layout *shape* (which window is in which dock node relative
  to the others) is the contract.
- **Window title regression.** The constants
  `default_layout_scene_window` etc. *are* the source of truth for
  the titles. D1 looks up window settings by `ImHashStr(constant)`
  via `FindWindowSettingsByID`. The implementer must also update
  `panels.cpp` (the `ImGui::Begin("Scene")` / `ImGui::Begin(
  "Inspector")` calls) and `main.cpp` (`ImGui::Begin("Gizmo", ...)`)
  to use the same constants — drift is caught at runtime because a
  panel-side rename without updating the constant would mean the
  constant points at a `WindowSettings` entry that has its `DockId`
  set, but the actual `Begin` call creates a *different*
  `ImGuiWindow` (under the new name) with no settings link, so the
  default layout would not apply on first launch. The smoke test
  catches this end-to-end; D1 catches it at the unit level (the
  named window has settings and a DockId — the implementer just has
  to follow through and call `Begin` with the same name).
- **Directory at the ini path.** `should_apply_default_layout` is
  documented as caller's-responsibility for a directory at the ini
  path; the predicate returns `false` (the path "exists"), the
  caller skips the default layout, and ImGui's later attempt to
  write to the path fails. Not separately tested — this is malformed
  caller input, not a contract.
- **XDG trailing slash** (`$XDG_CONFIG_HOME=/tmp/xdg/`). Declined:
  `std::filesystem::path` normalizes trailing-slash boundaries when
  joined, so the resolved path is correct on POSIX and Windows; the
  XDG spec does not require implementations to handle it. Not
  tested.

## Open follow-ups for the test-reviewer

1. **D1 leakiness.** Creating an ImGui context in a unit test is the
   first such test in the repo. The fixture (`DockLayoutTest`)
   isolates it. If the rev-2 reviewer prefers exercising this via a
   smoke-test rubric instead, the L-section tests still pin the
   pure-logic surface, but the structural contract (which window is
   in which leaf) reverts to "manual / code-review only." Author's
   read: keep D1 — it's the only automated check that catches
   "Inspector ended up in the central node" or "Gizmo got put in the
   right column." Cost is one test TU and a fixture; benefit is a
   regression net for the most user-visible part of the change.
2. **G1 grep brittleness.** The configure-time grep is text-based
   and locates a function body by string scan. If `main.cpp` is
   refactored to move `draw_gizmo_controls` into a separate TU, the
   grep needs an update. Documented in the lint script's preamble;
   author's read is acceptable cost. Reviewer can demand a
   tighter form if preferred (e.g. grep the entire `src/viz/` tree
   for `NoDocking` near the literal string `"Gizmo"`).
3. **Constants for the window names.** Putting `"Scene"` /
   `"Inspector"` / `"Gizmo"` in `layout.h` couples the layout module
   to the panel code by string. Alternative is to pass the names in
   from `main.cpp`. Author's read: a single shared header constant
   is the lowest-friction option and D1 catches drift via the
   DockBuilder name-lookup path.
