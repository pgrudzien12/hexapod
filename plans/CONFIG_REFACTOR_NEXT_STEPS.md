# Configuration Refactor - Next Steps & Recommendations

**Date:** January 22, 2026  
**Current Status:** Phase 3 In Progress (System namespace extraction)

---

## Current State Analysis

### ✅ What's Been Done

**Component Structure:**
```
components/
  config_core/
    include/config_core/
      manager.h       ✅ Registry API
      namespace.h     ✅ Descriptor definition
      types.h         ✅ Value union
    src/
      manager.c       ✅ Registry implementation
      access_api.c    ✅ (exists but needs work)
  
  config_ns_system/
    include/config_ns_system/
      system_namespace.h    ✅ Public API
      system_config.h       ✅ Config struct
    src/
      system_namespace.c    ✅ Partial implementation
    CMakeLists.txt          ✅ Component registration
  
  config_ns_joint_cal/
    include/config_ns_joint_cal/
      joint_cal_namespace.h  ✅ Headers
      joint_cal_config.h     ✅ Headers
    src/
      joint_cal_namespace.c  ✅ Exists (not fully migrated)
    CMakeLists.txt           ✅ Component registration
```

**Registry Implementation:**
- ✅ `config_core_register_namespace()` - works
- ✅ `config_core_get_namespace()` - works
- ✅ Basic descriptor structure defined
- ✅ Static registry array (max 8 namespaces)

**System Namespace:**
- ✅ NVS lifecycle functions extracted
  - `system_ns_init_defaults_nvs()`
  - `system_ns_load_from_nvs()`
  - `system_ns_save_to_nvs()`
  - `sys_load_defaults_mem()`
- ✅ NVS keys moved to component (internal)
- ✅ Isolated from main config_manager

### ⏳ What's Incomplete

**System Namespace:**
- ⏳ Descriptor instance not fully implemented
- ⏳ Not registered with core manager
- ⏳ Dynamic parameter interface hooks (NULL for now - OK)
- ⏳ API exports missing from header

**Joint Calibration:**
- ⏳ Still fully owned by `main/config_manager.c`
- ⏳ Parse functions not moved to component
- ⏳ NVS key generation not moved
- ⏳ Dynamic parameter interface not implemented

**Main config_manager.c:**
- ⏳ Still contains all joint_cal logic
- ⏳ Still owns global state for both namespaces
- ⏳ Still doing direct NVS operations
- ⏳ Dependencies on `"../../main"` in component CMakeLists

**Access API:**
- ⏳ `access_api.c` exists but not utilizing descriptors yet
- ⏳ Still calls into monolithic config_manager functions

---

## Issues Identified

### 🔴 Critical Issues

**1. CMakeLists Dependency Anti-Pattern**
```cmake
# In config_ns_system/CMakeLists.txt
INCLUDE_DIRS "include" "../../main"  # ❌ BAD: Component depends on main
```

**Problem:** Components should NOT depend on `main/`. This violates ESP-IDF component isolation principles.

**Impact:**
- `controller.h` is being accessed from `main/` for `controller_driver_type_e`
- Breaks component reusability
- Makes testing harder
- Violates architectural layering

**2. controller.h Dependency**
```c
// In system_config.h
#include "controller.h"  // For controller_driver_type_e
```

**Problem:** System configuration depends on controller types from main application.

### 🟡 Design Issues

**3. Incomplete Descriptor Implementation**

System namespace has implementation functions but:
```c
// Missing in system_namespace.c:
static const config_namespace_descriptor_t SYSTEM_NAMESPACE_DESCRIPTOR = {
    .name = "system",
    .schema_version = SYSTEM_SCHEMA_VERSION,
    // ... hooks filled in
};

// Missing registration function
esp_err_t system_ns_register_with_core(void *cache);
```

**4. Global State Still in main/**

```c
// In main/config_manager.c - should move to component private state
static system_config_t g_system_config = {0};
static joint_calib_config_t g_joint_calib_config = {0};
```

**5. Incomplete Phase 3**

According to the plan, Phase 3 should:
- Move system-specific tables → ✅ Done
- Register descriptor with core → ❌ Not done
- Replace direct calls with component functions → ❌ Partial

---

## Recommended Next Commit

### Priority 1: Fix Controller Type Dependency 🔴

**Problem:** `controller_driver_type_e` is in `main/controller.h`

**Solutions (pick one):**

**Option A: Extract Controller Types (RECOMMENDED)**
```c
// Create: components/config_core/include/config_core/controller_types.h
#pragma once

typedef enum {
    CONTROLLER_DRIVER_FLYSKY_IBUS = 0,
    CONTROLLER_DRIVER_WIFI_TCP = 1,
    CONTROLLER_DRIVER_BT_CLASSIC = 2,
    CONTROLLER_DRIVER_COUNT
} controller_driver_type_e;
```

Then:
- Update `main/controller.h` to include `config_core/controller_types.h`
- Update `system_config.h` to include `config_core/controller_types.h`
- Remove `"../../main"` from `config_ns_system/CMakeLists.txt`

**Option B: Use Generic Integer**
```c
// In system_config.h
typedef uint8_t controller_driver_type_t;  // 0=FlySky, 1=WiFi, 2=BT

// Define constants
#define CONTROLLER_TYPE_FLYSKY_IBUS  0
#define CONTROLLER_TYPE_WIFI_TCP     1
#define CONTROLLER_TYPE_BT_CLASSIC   2
```

Less type-safe but breaks circular dependency.

**Option C: Remove Controller Type from System Config**

Move controller selection to a separate namespace (`config_ns_controller` in future).

**Recommendation:** Use Option A - cleanest separation.

### Priority 2: Complete System Namespace Descriptor

**File: `components/config_ns_system/src/system_namespace.c`**

Add at end of file:

```c
// Static descriptor instance
static const config_namespace_descriptor_t s_system_descriptor = {
    .name = "system",
    .schema_version = SYSTEM_SCHEMA_VERSION,
    .runtime_cache_ptr = NULL,  // Managed externally for now (Phase 3)
    .runtime_cache_size = sizeof(system_config_t),
    
    // Lifecycle hooks
    .load_defaults_mem = sys_load_defaults_mem,
    .init_defaults_nvs = system_ns_init_defaults_nvs,
    .load_from_nvs = system_ns_load_from_nvs,
    .save_to_nvs = system_ns_save_to_nvs,
    .migrate_step = NULL,  // Optional, add later
    
    // Dynamic parameter interface (Phase 4+)
    .list_param_names = NULL,
    .find_param = NULL,
    .get_value = NULL,
    .set_value = NULL,
    .validate_value = NULL
};

const config_namespace_descriptor_t* system_ns_get_descriptor(void)
{
    return &s_system_descriptor;
}

esp_err_t system_ns_register_with_core(void)
{
    return config_core_register_namespace(&s_system_descriptor);
}
```

**File: `components/config_ns_system/include/config_ns_system/system_namespace.h`**

Add exports:

```c
// Get descriptor for registration
const config_namespace_descriptor_t* system_ns_get_descriptor(void);

// Convenience registration helper
esp_err_t system_ns_register_with_core(void);
```

### Priority 3: Update main/config_manager.c to Use Registry

**In `config_manager_init()`:**

```c
// After NVS initialization
esp_err_t config_manager_init(void) {
    // ... existing partition init ...
    
    // Register system namespace
    ESP_ERROR_CHECK(system_ns_register_with_core());
    ESP_LOGI(TAG, "System namespace registered");
    
    // Open system namespace NVS handle
    nvs_handle_t system_handle = 0;
    esp_err_t err = nvs_open_from_partition(NVS_PARTITION_ROBOT, "system", 
                                             NVS_READWRITE, &system_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open system namespace: %s", esp_err_to_name(err));
        return err;
    }
    
    // Get descriptor from registry
    const config_namespace_descriptor_t *sys_desc = NULL;
    err = config_core_get_namespace("system", &sys_desc);
    if (err != ESP_OK || !sys_desc) {
        ESP_LOGE(TAG, "System namespace not registered!");
        nvs_close(system_handle);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Check if namespace needs initialization
    uint16_t stored_version = 0;
    err = nvs_get_u16(system_handle, SYS_KEY_CONFIG_VERSION, &stored_version);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot - initialize defaults
        ESP_LOGI(TAG, "System namespace not initialized, creating defaults");
        if (sys_desc->init_defaults_nvs) {
            ESP_ERROR_CHECK(sys_desc->init_defaults_nvs(system_handle));
        }
    }
    // TODO: Add migration if stored_version < sys_desc->schema_version
    
    // Load defaults into memory cache first
    if (sys_desc->load_defaults_mem) {
        ESP_ERROR_CHECK(sys_desc->load_defaults_mem(&g_system_config));
    }
    
    // Load persisted values from NVS (overlays defaults)
    if (sys_desc->load_from_nvs) {
        ESP_ERROR_CHECK(sys_desc->load_from_nvs(system_handle, &g_system_config));
    }
    
    nvs_close(system_handle);
    g_manager_state.initialized = true;
    
    // TODO: Repeat for joint_cal namespace once extracted
    
    return ESP_OK;
}
```

---

## Commit Strategy

### Commit 1: Fix Controller Type Dependency ✅ DO THIS NEXT

**Goal:** Break circular dependency between config_core and main/

**Changes:**
1. Create `components/config_core/include/config_core/controller_types.h`
2. Move `controller_driver_type_e` definition
3. Update `main/controller.h` to include the new header
4. Update `config_ns_system/include/config_ns_system/system_config.h`
5. Remove `"../../main"` from `config_ns_system/CMakeLists.txt`
6. Update `INCLUDE_DIRS "include"` only

**Verification:**
- Build passes
- No warnings about includes
- Component independence verified

**Commit Message:**
```
refactor(config): extract controller types to break circular dependency

- Move controller_driver_type_e to config_core/controller_types.h
- Remove main/ dependency from config_ns_system component
- Improve component isolation and reusability

Ref: CONFIG_MANAGER_REFACTOR_PLAN.md Phase 3
```

### Commit 2: Complete System Namespace Descriptor

**Goal:** Fully implement descriptor pattern for system namespace

**Changes:**
1. Add descriptor instance in `system_namespace.c`
2. Add `system_ns_get_descriptor()` function
3. Add `system_ns_register_with_core()` helper
4. Export functions in `system_namespace.h`
5. Add `#include "config_core/manager.h"` for registration

**Verification:**
- Build passes
- Descriptor can be retrieved by name
- All hooks properly wired

**Commit Message:**
```
refactor(config): complete system namespace descriptor implementation

- Add static descriptor with lifecycle hooks
- Export registration helper functions
- Enable descriptor-based namespace management

Ref: CONFIG_MANAGER_REFACTOR_PLAN.md Phase 3
```

### Commit 3: Use Descriptor in config_manager

**Goal:** Replace direct system namespace calls with descriptor-based approach

**Changes:**
1. Update `config_manager_init()` to use registry
2. Call `system_ns_register_with_core()` during init
3. Use descriptor hooks instead of direct function calls
4. Keep legacy public API intact (backward compatibility)

**Verification:**
- Build and flash
- Verify NVS initialization works
- Test parameter get/set operations
- Verify migration on version mismatch (if implemented)

**Commit Message:**
```
refactor(config): use descriptor-based system namespace in manager

- Register system namespace on init
- Use descriptor hooks for lifecycle operations
- Remove direct function calls to namespace implementation
- Maintain backward compatibility in public API

Ref: CONFIG_MANAGER_REFACTOR_PLAN.md Phase 3, checkpoint
```

---

## After These Commits: Next Phase

### Then: Extract Joint Calibration (Phase 4)

Similar process:
1. Move joint parsing functions to `config_ns_joint_cal/src/`
2. Move NVS key generation to component
3. Create descriptor with dynamic parameter hooks
4. Register and use descriptor in manager

### Then: Global State Management (Phase 5)

- Move `g_system_config` to system namespace component (private)
- Move `g_joint_calib_config` to joint_cal component (private)
- Expose via accessor functions
- Update config_manager to delegate to components

---

## Testing Checklist

After each commit:

- [ ] `idf.py build` succeeds with no warnings
- [ ] Flash to device and verify boot
- [ ] Check NVS initialization on first boot
- [ ] Test parameter get operations
- [ ] Test parameter set operations (memory)
- [ ] Test parameter set operations (persist)
- [ ] Test save namespace operation
- [ ] Test factory reset
- [ ] Verify no regressions in RPC commands
- [ ] Check memory usage (heap/stack)

---

## Long-term Cleanups (Future Commits)

1. **Remove legacy get/set functions** - Use descriptor-based access throughout
2. **Implement dynamic parameter interface** - For RPC auto-discovery
3. **Extract remaining namespaces** - motion_lim, servo_map, gait, etc.
4. **Auto-registration via linker sections** - Remove manual registration
5. **Move config_manager.c to config_core** - Complete componentization

---

## Risk Mitigation

### Backward Compatibility

**Current Public API Must Keep Working:**
```c
// These functions in config_manager.h must continue to work
esp_err_t config_manager_init(void);
bool config_manager_has_dirty_data(void);
esp_err_t config_manager_save_namespace(config_namespace_t ns);

// System namespace getters/setters
const system_config_t* config_get_system(void);
esp_err_t config_set_system_emergency_stop_memory(bool enabled);
// ... etc

// Joint calibration getters/setters
const joint_calib_config_t* config_get_joint_calib();
esp_err_t config_get_joint_calib_data(int leg_index, int joint_index, joint_calib_data_t* data);
// ... etc
```

**Strategy:**
- Keep all public functions as thin wrappers
- Internal implementation uses descriptors
- Add new descriptor-based functions alongside (don't replace yet)
- Deprecate old API in Phase 7+

### Testing Between Phases

**Checkpoint tests after each major change:**
1. NVS operations (init, read, write, delete)
2. Configuration persistence across reboots
3. RPC command integration
4. Performance (no significant regression)
5. Memory usage (no leaks or excessive growth)

---

## Summary

### Next Immediate Actions

1. **THIS COMMIT:** Fix controller type dependency
   - Extract `controller_driver_type_e` to `config_core/controller_types.h`
   - Remove `../../main` dependency from components
   - Verify build succeeds

2. **NEXT COMMIT:** Complete system namespace descriptor
   - Add descriptor instance with hooks
   - Export registration functions
   - Wire up in header

3. **COMMIT AFTER THAT:** Use descriptor in manager
   - Register system namespace
   - Call via descriptor hooks
   - Maintain legacy API compatibility

### Success Criteria

After these 3 commits, you should have:
- ✅ System namespace fully extracted and working via descriptor
- ✅ No `../../main` dependencies in components
- ✅ Registry pattern working end-to-end
- ✅ Foundation ready for joint_cal extraction
- ✅ All existing functionality still works
- ✅ Clean component architecture

**This sets you up perfectly for Phase 4 (joint_cal extraction).**

---

*Prepared: January 22, 2026*
