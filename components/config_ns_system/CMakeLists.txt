idf_component_register(
    SRCS "src/system_namespace.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_system config_core
)
