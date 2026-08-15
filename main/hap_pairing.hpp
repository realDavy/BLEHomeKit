#pragma once

#include "hap/platform/Storage.hpp"

// Hold encoder SW (GPIO9) at power-on to clear leftover HomeKit pairings
// without changing the MAC-derived setup code.
bool hap_encoder_sw_held(int hold_ms);

// Log whether a leftover pairing_list is present. Pairings persist across reboot.
void hap_wipe_legacy_nvs();

// Remove controller pairings and GSN. Keeps accessory_id / LTSK so the
// setup code and identity stay the same.
void hap_clear_controller_pairings(hap::platform::Storage& storage);

// Drop empty, junk, or incomplete pairing_list entries that would make
// HAP advertise SF=0 (already paired) so iPhone will not show the accessory.
// Returns true if a verified controller pairing remains (advertise SF=0).
// Pair-Setup without Pair-Verify still returns false so Home can rediscover.
bool hap_sanitize_pairings(hap::platform::Storage& storage);

// Make the public BLE MAC match the HAP Device ID (or write the factory
// BT MAC as Device ID). Must run before NimBLE init. Returns false if
// pairings were cleared because the identity had to change.
bool hap_align_ble_identity(hap::platform::Storage& storage);
