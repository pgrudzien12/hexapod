idf_component_register(
    SRCS "src/joint_cal_namespace.c"
    INCLUDE_DIRS "include" "../../main"  # TODO: extract joint_types.h to remove this dependency
    REQUIRES nvs_flash config_core
)
