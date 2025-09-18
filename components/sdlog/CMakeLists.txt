idf_component_register(
    SRCS "sdlog.c"
    INCLUDE_DIRS "include"
    REQUIRES vfs fatfs sdmmc driver
)