idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES gps motor solar tracking sdlog status_led button
)