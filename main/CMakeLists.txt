idf_component_register(
    SRCS
        "main.cpp"
        "light_controller.cpp"
        "light_ui.cpp"
        "encoder_input.cpp"
    INCLUDE_DIRS "."
    REQUIRES hap_wrapper yahap-pal esp_event nvs_flash driver esp_timer led_strip knob esp32_c3_lcdkit
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
