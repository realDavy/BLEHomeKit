set(srcs
    "Esp32Platform.cpp"
    "Esp32Storage.cpp"
    "Esp32Crypto.cpp"
    "Esp32Ble.cpp"
)
set(priv_requires nvs_flash esp_event mbedtls libsodium)

# A hard REQUIRES esp_wifi forces CONFIG_ESP_WIFI_ENABLED=y even for BLE-only
# builds. Only pull Wi-Fi / mDNS when HAP-IP is actually enabled.
if(CONFIG_PAL_NETWORK_ENABLED)
    list(APPEND srcs "Esp32Network.cpp")
    list(APPEND priv_requires esp_wifi mdns)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES hap_wrapper bt esp_timer
    PRIV_REQUIRES ${priv_requires}
)
