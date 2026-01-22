# Configuration Manager Refactor Plan

## 1. Problem Summary
The current `config_manager.c` is a monolith handling:
- Partition + NVS init
- Global + per-namespace migration
- Namespace defaults (system + joint calibration)
- Load/save logic
- Parameter metadata tables
- Per-type get/set APIs and RPC listing helpers

As new namespaces are added this design causes:
- High coupling (central edits required in multiple places)
- Memory growth (large static tables, especially joint parameters)
- Harder testing and incremental migration

## 2. Goals
1. Add a new namespace by creating only a new pair of files (descriptor + implementation) — no central file edits after stabilization.
2. Eliminate externally exposed static parameter tables; allow dynamic generation (important for many leg×joint parameters).
3. Maintain existing public APIs initially for compatibility (phase migration).
4. Support per-namespace schema versioning & migration steps.
5. Prepare for componentization inside ESP-IDF (`components/` layout).
6. No "dumping ground" headers; keep namespace logic encapsulated.

## 3. High-Level Architecture
```
components/
  config_core/
    include/config_core/manager.h        # Public init + legacy API façade
    include/config_core/namespace.h      # Descriptor definition & registry API
    include/config_core/types.h          # Shared enums + value union
    src/manager.c                        # Init, iteration, global orchestration
    src/access_api.c                     # hexapod_config_get_* routing via descriptors
    src/migration.c                      # Optional global coordination
  config_ns_system/
    include/config_ns_system/system_config.h
    src/system_namespace.c               # Descriptor + implementation
  config_ns_joint_cal/
    include/config_ns_joint_cal/joint_cal_config.h
    src/joint_cal_namespace.c            # Descriptor + dynamic param generation
  config_ns_<future>/
```

## 4. Namespace Descriptor (Dynamic Parameter Model)
```c
typedef union {
    bool     b;
    int32_t  i32;
    uint32_t u32;
    uint16_t u16;
    float    f;
    char     s[64]; // transient buffer for small strings
} config_value_union_t;

typedef struct {
    const char *name;                     // "system", "joint_cal" ...
    uint16_t schema_version;              // Per-namespace schema version
    void *runtime_cache_ptr;              // RAM struct (optional)
    size_t runtime_cache_size;

    // Lifecycle & persistence
    esp_err_t (*load_defaults_mem)(void *cache);
    esp_err_t (*init_defaults_nvs)(nvs_handle_t h);
    esp_err_t (*load_from_nvs)(nvs_handle_t h, void *cache);
    esp_err_t (*save_to_nvs)(nvs_handle_t h, const void *cache);
    esp_err_t (*migrate_step)(nvs_handle_t h, uint16_t from_ver, uint16_t to_ver); // optional

    // Dynamic parameter interface
    esp_err_t (*list_param_names)(size_t offset, size_t max, const char **names_out, size_t *returned);
    esp_err_t (*find_param)(const char *name, config_param_info_t *out_meta); // fills type + constraints
    esp_err_t (*get_value)(const char *name, config_value_union_t *out_val);
    esp_err_t (*set_value)(const char *name, const config_value_union_t *in_val, bool persist);
    esp_err_t (*validate_value)(const char *name, const config_value_union_t *candidate); // optional
} config_namespace_descriptor_t;
```
Internal static tables (e.g., system params) remain private within their namespace implementation for Phase 1.

## 5. Lifecycle Flow (Init)
For each registered descriptor:
1. Open its NVS namespace handle.
2. Read per-namespace version key (`config_ver`).
3. If missing: `init_defaults_nvs()` → write version.
4. If older: loop `migrate_step(from, from+1)` until current.
5. `load_defaults_mem()` into cache.
6. `load_from_nvs()` overlay actual persisted values.

Global version becomes optional; migration becomes per-namespace.

## 6. Parameter Access Refactor
Legacy APIs (`hexapod_config_get_bool`, etc.):
1. Map namespace string → descriptor.
2. Call `find_param(name, &meta)` to verify existence + type.
3. Call `get_value(name, &val)`; translate union to requested type.
4. `set_*` does reverse, invokes `set_value()`.

RPC listing uses new helper `config_list_parameters(ns, offset, max, names_out, &returned)` which simply calls `descriptor->list_param_names()`.

## 7. Adding a New Namespace (After Stabilization)
Steps:
1. Create `components/config_ns_<new>/include/.../<new>_config.h` (types + public struct if needed).
2. Implement `src/<new>_namespace.c` with descriptor + functions.
3. Register descriptor (Phase 1: add to central array; Phase 4+: auto-registration via linker section or constructor function).
4. (Optional) Add migration steps inside `migrate_step()`.
No edits in `config_core` required once auto-registration is in place.

## 8. Migration Strategy
- Each namespace stores own version.
- `migrate_step()` handles only `v -> v+1`.
- Manager loops until `stored == schema_version`.
- Failure aborts init with logged error.

## 9. Phased Refactor Plan

Phase 1 — Scaffold Core Interfaces (no behavioral change)
- Add `components/config_core/include/config_core/namespace.h` with the descriptor type and value union.
- Add `components/config_core/include/config_core/manager.h` with forward declarations mirroring current public API.
- No code moved yet; `config_manager.c` remains the single implementation.
- Goal: compile passes with headers available; zero runtime change.

Phase 2 — Introduce Core Adapter Layer (minimal routing)
- Create `components/config_core/src/access_api.c` implementing thin wrappers that call existing functions in `config_manager.c` (temporary).
- Define a minimal in-process descriptor array in `config_manager.c` (still local) to prove the pattern without splitting.
- Keep all namespace logic (system, joint_cal) in `config_manager.c`.
- Checkpoint: build succeeds; unit tests pass; behavior unchanged.

Phase 3 — Extract System Namespace Component
- Create `components/config_ns_system/` with:
  - `include/config_ns_system/system_config.h` (types already exist; move non-generic bits here).
  - `src/system_namespace.c` implementing: `load_defaults_mem`, `init_defaults_nvs`, `load_from_nvs`, `save_to_nvs` (dynamic param hooks can remain NULL in Phase 3).
  - Define and expose `SYSTEM_NAMESPACE_DESCRIPTOR` and helpers `system_ns_get_descriptor()` and `system_ns_register_with_core()`.
- Move system-specific tables and keys from `config_manager.c` into `system_namespace.c` (private scope).
- In `config_manager.c`, replace direct calls with component functions and register the descriptor with the core minimal registry.
- Checkpoint: only system namespace is fully migrated to component; joint_cal still monolithic.

Phase 4 — Extract Joint Calibration Namespace Component (dynamic params)
- Create `components/config_ns_joint_cal/` with:
  - `include/config_ns_joint_cal/joint_cal_config.h` (public types).
  - `src/joint_cal_namespace.c` implementing dynamic `list_param_names`, `find_param` via existing `parse_joint_param_name`, plus load/save/defaults.
- Move joint calibration cache (`g_joint_calib_config`) and all NVS key formats out of `config_manager.c` into this component.
- Checkpoint: both namespaces live in components; `config_manager.c` keeps global lifecycle and descriptor wiring.

Phase 5 — Move Global Lifecycle into Config Core
- Create `components/config_core/src/manager.c` implementing:
  - Partition initialization (default + robot NVS).
  - Descriptor iteration for per-namespace init, migration sequencing, and cache loads.
  - Dirty tracking and `config_manager_save_namespace` routing via descriptors.
- Trim `main/config_manager.c` to a thin façade or completely remove it if all public APIs are provided by `config_core`.
- Checkpoint: All functionality resides under `components/config_core` + `components/config_ns_*`; `main/` contains no configuration logic.

Phase 6 — Descriptor Registration Simplification
- Initial: minimal runtime registry implemented in `config_core/manager.c` providing `config_core_register_namespace()` and `config_core_get_namespace()`.
- Register descriptors from each `config_ns_*` via a helper (e.g., `system_ns_register_with_core()`), avoiding central edits.
- Optional upgrade: switch to linker section or constructor-based auto-registration to eliminate any registration calls when adding new namespaces.
- Checkpoint: Adding new namespace requires only creating `components/config_ns_new/` with its descriptor; no edits elsewhere (post-upgrade).

Phase 7 — Public API Routing and Deprecation
- Route all `hexapod_config_get_*`/`set_*` calls through `config_core/src/access_api.c`, which resolves descriptors and uses `find_param` + `get_value`/`set_value`.
- Remove any direct offset arithmetic or exposed static tables from public headers.
- Keep backwards compatibility by preserving function signatures; mark legacy metadata fields (e.g., description) as optional/unused.
- Checkpoint: RPC (`rpc_commands.c`) continues working via `config_get_parameter_info` and `config_list_parameters` implemented on top of descriptors.

Phase 8 — Performance/UX Enhancements (optional)
- Add pagination parameters to listing APIs used by RPC for very large sets.
- Consider hash-based lookup caches inside namespaces if linear `find_param` becomes hot.
- Remove deprecated fields from internal structs once RPC relies solely on type + constraints.

Completion Criteria — "All code migrated"
- All namespace-specific code resides in `components/config_ns_*`.
- All lifecycle, routing, save/load orchestration resides in `components/config_core`.
- `main/` no longer contains configuration logic; only includes and usage.
- New namespace addition requires only a new component folder with descriptor + implementation (no edits in core post-registration upgrade).

## 10. Risks & Mitigations
| Risk | Mitigation |
|------|------------|
| Large dynamic listing cost | Support offset + max pagination; avoid building full lists. |
| String buffer lifetime issues | Caller-owned buffers for string get/set; union only for primitives. |
| Migration partial failure | Abort init early; provide factory-reset fallback. |
| Descriptor registration order | Use deterministic array first; later adopt linker section scanning. |
| Performance overhead of indirection | Param counts modest; optimize later with hash map if needed. |
| Backward compatibility | Keep legacy public API until Phase 3 complete; deprecate gradually. |

## 11. Testing Strategy
- Namespace unit tests: defaults, load/save, migration from prior version, constraints enforcement.
- Integration test: full init after flash erase (factory-new scenario).
- RPC tests: list/get/set round-trips for representative parameters.
- Joint calibration parse tests for edge cases (invalid leg/joint, out-of-range values).

## 12. Documentation Impact
- Parameter descriptions removed from firmware; maintained in Markdown docs.
- This file serves as the living spec for the refactor until merged; after completion integrate into existing design docs.

## 13. Next Immediate Actions
1. Wire access API to use registry lookups for save/get/set where feasible (keep fallbacks).  
2. Extract joint calibration namespace into `components/config_ns_joint_cal` (defaults, load/save, descriptor).  
3. Register joint calibration descriptor during init, replace monolith calls.  
4. Begin per-namespace migration versioning design and keys.  

---
Prepared: 2025-11-23
