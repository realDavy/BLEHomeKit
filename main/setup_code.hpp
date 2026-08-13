#pragma once

#include <cstdint>
#include <string>

/** 8-digit HomeKit setup code "XXX-XX-XXX" derived from the factory MAC. */
std::string hap_setup_code_from_mac();

/** Factory MAC as "AABBCCDDEEFF" for the Accessory serial number. */
std::string hap_serial_from_mac();
