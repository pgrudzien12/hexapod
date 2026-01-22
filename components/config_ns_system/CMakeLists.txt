idf_component_register(
    SRCS "src/system_namespace.c"
    INCLUDE_DIRS "include" "../../main"  # access controller.h until extracted
    REQUIRES nvs_flash esp_system config_core          # esp_read_mac provided by esp_system; config_core for headers
)
