# AppConfig component - NVS-backed runtime settings with Kconfig fallbacks

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/AppConfig.cpp"
        "src/AppConfigAccess.cpp"
        "src/AppConfigDefaultsCell.cpp"
        "src/AppConfigDefaultsMotion.cpp"
        "src/AppConfigParamIo.cpp"
        "src/AppConfigText.cpp"
        "src/ConfigSchema.cpp"
        "src/NvsStorage.cpp"
    INCLUDE_DIRS
        "src"
        "${APP_ROOT}/include"
    REQUIRES
        nvs_flash
        esp_common
        log
)
