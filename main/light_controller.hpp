#pragma once

#include <cstdint>
#include <functional>
#include <mutex>

/**
 * LED dimming state used by both the rotary encoder and HomeKit.
 *
 * Color mixing follows the official knob_panel light example:
 *   cool: R=G=B = pwm
 *   warm: R=G = pwm, B = pwm * 0x33 / 0xFF
 */
class LightController {
public:
    struct State {
        bool on = true;
        int brightness = 50;            // 0-100, same default as knob_panel
        uint32_t color_temp_mireds = 370; // warm white
    };

    using Listener = std::function<void(const State& state)>;

    static LightController& instance();

    void init();
    void set_on(bool on);
    void set_brightness(int brightness);
    void set_color_temp(uint32_t mireds);
    void toggle_color_temp();
    void encoder_brighter();
    void encoder_dimmer();
    void identify();

    State state() const;
    void set_listener(Listener listener);

private:
    LightController() = default;

    void apply_locked();
    void compute_rgb_locked(uint8_t& r, uint8_t& g, uint8_t& b) const;
    static int snap_up(int brightness);
    static int snap_down(int brightness);

    mutable std::mutex mutex_;
    State state_{};
    Listener listener_;
    bool led_ready_ = false;
};
