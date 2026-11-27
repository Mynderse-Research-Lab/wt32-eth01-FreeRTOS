# AppConfig component - NVS-backed runtime settings with Kconfig fallbacks

set(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")

idf_component_register(
    SRCS
        "src/AppConfig.cpp"
        "src/AppConfigAccess.cpp"
        "src/schema/AppConfigDefaultsCell.cpp"
        "src/schema/AppConfigDefaultsMotion.cpp"
        "src/AppConfigParamIo.cpp"
        "src/AppConfigText.cpp"
        "src/schema/ConfigSchema.cpp"
        "src/storage/NvsStorage.cpp"
    INCLUDE_DIRS
        "src"
        "src/schema"
        "src/storage"
        "${APP_ROOT}/include"
    REQUIRES
        nvs_flash
        esp_common
        log
)
