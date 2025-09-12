idf_component_register(
    SRCS "tracking.c"
    INCLUDE_DIRS "." "../../include"
    REQUIRES gps solar motor battery freertos esp_system
)