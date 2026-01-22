idf_component_register(
    SRCS "src/joint_cal_namespace.c"
    INCLUDE_DIRS "include" "../../main"  # temporary access to joint_types.h or other legacy includes
    REQUIRES nvs_flash config_core
)
