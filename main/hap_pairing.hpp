#pragma once

#include "hap/platform/Storage.hpp"

// Hold encoder SW (GPIO9) at power-on to clear leftover HomeKit pairings
// without changing the MAC-derived setup code.
bool hap_encoder_sw_held(int hold_ms);

// Delete pairing_list at every boot so HomeKit advertises SF=1.
void hap_wipe_legacy_nvs();

// Remove controller pairings and GSN. Keeps accessory_id / LTSK so the
// setup code and identity stay the same.
void hap_clear_controller_pairings(hap::platform::Storage& storage);

// Drop empty, junk, or incomplete pairing_list entries that would make
// HAP advertise SF=0 (already paired) so iPhone will not show the accessory.
// Returns true if a valid controller pairing remains.
bool hap_sanitize_pairings(hap::platform::Storage& storage);
