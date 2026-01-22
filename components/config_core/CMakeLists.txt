idf_component_register(
    SRCS
        "src/access_api.c"
        "src/manager.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
