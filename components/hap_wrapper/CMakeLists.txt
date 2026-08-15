# Compile realDavy/HAP as this IDF component.
# Do not add_subdirectory(hap): HAP's CMakeLists.txt also calls
# idf_component_register() and FetchContent(nlohmann/json), which breaks
# ESP-IDF 5.5 (duplicate __idf_hap_wrapper + FetchContent URL error).

idf_component_register(
    SRCS
        "AccessoryServer.cpp"  # overlay: unpair keeps Device ID / BLE address
        "hap/src/common/TaskScheduler.cpp"
        "hap/src/core/TLV8.cpp"
        "hap/src/core/AttributeDatabaseJSON.cpp"
        "hap/src/core/CharacteristicSerializer.cpp"
        "hap/src/core/CharacteristicFinder.cpp"
        "hap/src/core/IIDManager.cpp"
        "hap/src/transport/HTTP.cpp"
        "hap/src/transport/SecureSession.cpp"
        "hap/src/transport/Router.cpp"
        "hap/src/platform/Ble.cpp"
        "BleTransport.cpp"  # overlay: GATT fragments, SF until Pair-Verify, no empty FE59 GATT
        "hap/src/transport/ConnectionContext.cpp"
        "hap/src/transport/PairingEndpoints.cpp"
        "hap/src/transport/AccessoryEndpoints.cpp"
        "hap/src/transport/ble/HapPdu.cpp"
        "hap/src/transport/ble/BleTlvBuilder.cpp"
        "hap/src/transport/ble/BleSessionManager.cpp"
        "hap/src/pairing/PairSetup.cpp"
        "hap/src/pairing/PairVerify.cpp"
        "hap/src/types/CharacteristicTypes.cpp"
        "hap/src/types/ServiceTypes.cpp"
    INCLUDE_DIRS
        "include_override"
        "hap/include"
    PRIV_REQUIRES
        johboh__nlohmann-json
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
target_compile_definitions(${COMPONENT_LIB} PRIVATE JSON_NOEXCEPTION)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-pedantic
    -Wno-missing-field-initializers
)
