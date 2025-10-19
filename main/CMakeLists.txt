idf_component_register(
    SRCS
    "robot_config_fat.c"
    "main.c"
    "leg.c"
    "gait_scheduler.c"
    "swing_trajectory.c"
    "whole_body_control.c"
    "robot_control.c"
    "robot_config.c"
    "user_command.c"
    "controller.c"
    "controller_flysky_ibus.c"
    "controller_wifi_tcp.c"
    "controller_bt_classic.c"
    "wifi_ap.c"
    INCLUDE_DIRS "."
)
