#include "light_ui.hpp"
#include "board_pins.hpp"

#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"


static const char* TAG = "light_ui";

static lv_obj_t* s_arc = nullptr;
static lv_obj_t* s_percent = nullptr;
static lv_obj_t* s_mode = nullptr;
static lv_obj_t* s_status = nullptr;
static lv_obj_t* s_setup = nullptr;
static bool s_paired = false;
static bool s_started = false;

static lv_color_t color_for_temp(uint32_t mireds) {
    const bool warm = mireds >= ((LIGHT_WARM_MIREDS + LIGHT_COOL_MIREDS) / 2);
    return warm ? lv_color_hex(0xFFB347) : lv_color_hex(0x7EC8E3);
}

static void render_state(const LightController::State& state) {
    if (!s_started || s_arc == nullptr) {
        return;
    }
    const int shown = state.on ? state.brightness : 0;
    const bool warm = state.color_temp_mireds >= ((LIGHT_WARM_MIREDS + LIGHT_COOL_MIREDS) / 2);
    lv_arc_set_value(s_arc, shown);
    lv_obj_set_style_arc_color(s_arc, color_for_temp(state.color_temp_mireds), LV_PART_INDICATOR);

    if (shown == 0) {
        lv_label_set_text(s_percent, "--");
    } else {
        lv_label_set_text_fmt(s_percent, "%d%%", shown);
    }
    lv_label_set_text(s_mode, warm ? "Warm" : "Cool");

    if (s_paired) {
        lv_label_set_text(s_status, "HomeKit paired");
        lv_obj_add_flag(s_setup, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_label_set_text(s_status, "HomeKit BLE");
        lv_obj_clear_flag(s_setup, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_setup, "PIN " HAP_SETUP_CODE);
    }
}

void light_ui_start() {
    auto* disp = bsp_display_start();
    if (disp == nullptr) {
        ESP_LOGW(TAG, "Display init failed, continuing without UI");
        return;
    }
    (void)disp;

    bsp_display_lock(0);

    lv_obj_t* scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    s_arc = lv_arc_create(scr);
    lv_obj_set_size(s_arc, 200, 200);
    lv_obj_center(s_arc);
    lv_arc_set_rotation(s_arc, 135);
    lv_arc_set_bg_angles(s_arc, 0, 270);
    lv_arc_set_range(s_arc, 0, 100);
    lv_obj_remove_style(s_arc, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(s_arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_arc_width(s_arc, 14, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_arc, 14, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_arc, lv_color_hex(0x2A3038), LV_PART_MAIN);

    s_percent = lv_label_create(scr);
    lv_obj_set_style_text_font(s_percent, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_percent, lv_color_hex(0xF5F5F5), 0);
    lv_obj_align(s_percent, LV_ALIGN_CENTER, 0, -8);

    s_mode = lv_label_create(scr);
    lv_obj_set_style_text_font(s_mode, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_mode, lv_color_hex(0xC8C8C8), 0);
    lv_obj_align(s_mode, LV_ALIGN_CENTER, 0, 28);

    s_status = lv_label_create(scr);
    lv_obj_set_style_text_font(s_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_status, lv_color_hex(0x8AA0B8), 0);
    lv_obj_align(s_status, LV_ALIGN_BOTTOM_MID, 0, -28);

    s_setup = lv_label_create(scr);
    lv_obj_set_style_text_font(s_setup, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_setup, lv_color_hex(0xFFE08A), 0);
    lv_obj_align(s_setup, LV_ALIGN_BOTTOM_MID, 0, -10);

    s_started = true;
    render_state(LightController::instance().state());
    bsp_display_unlock();

    bsp_display_backlight_on();
    ESP_LOGI(TAG, "Light UI started");
}

void light_ui_set_paired(bool paired) {
    s_paired = paired;
    light_ui_refresh();
}

void light_ui_refresh() {
    if (!s_started) {
        return;
    }
    if (!bsp_display_lock(50)) {
        return;
    }
    render_state(LightController::instance().state());
    bsp_display_unlock();
}
