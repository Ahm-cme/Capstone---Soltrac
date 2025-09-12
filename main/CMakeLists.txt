idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../include"
    REQUIRES button gps battery motor solar tracking
)