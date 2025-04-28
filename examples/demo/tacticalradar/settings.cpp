#include "settings.h"
#include <lvgl.h>

static LilyGoLib* _watch = nullptr;
static int* _threshold = nullptr;
static std::vector<SignalSource>* _history = nullptr;

static lv_obj_t* settings_screen = nullptr;

// Save detection threshold slider change
static void save_threshold(lv_event_t* e) {
    lv_obj_t* slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    if (_threshold) {
        *_threshold = val;
    }
}

// Close settings UI
static void close_settings(lv_event_t* e) {
    lv_scr_load(lv_scr_act()); // Go back to radar
}

// Open Settings Screen
void settings_open(LilyGoLib** watch, int* threshold, std::vector<SignalSource>* history) {
    _watch = *watch;
    _threshold = threshold;
    _history = history;

    settings_screen = lv_obj_create(NULL);
    lv_scr_load(settings_screen);
    lv_obj_set_style_bg_color(settings_screen, lv_color_black(), LV_PART_MAIN);

    // Title
    lv_obj_t* title = lv_label_create(settings_screen);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Detection Threshold
    lv_obj_t* label1 = lv_label_create(settings_screen);
    lv_label_set_text(label1, "Detection Threshold:");
    lv_obj_set_style_text_color(label1, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label1, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t* slider = lv_slider_create(settings_screen);
    lv_obj_set_width(slider, 180);
    lv_slider_set_range(slider, -100, -30);
    lv_slider_set_value(slider, *_threshold, LV_ANIM_OFF);
    lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_add_event_cb(slider, save_threshold, LV_EVENT_VALUE_CHANGED, NULL);

    // Recent Detections List
    lv_obj_t* label2 = lv_label_create(settings_screen);
    lv_label_set_text(label2, "Recent Detections:");
    lv_obj_set_style_text_color(label2, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(label2, LV_ALIGN_TOP_MID, 0, 110);

    int y = 140;
    int count = 0;
    for (auto it = _history->rbegin(); it != _history->rend(); ++it) {
        if (count++ >= 5) break; // Show last 5 only

        String txt = it->source + " (" + it->type + " / " + it->manufacturer + ")";
        lv_obj_t* entry = lv_label_create(settings_screen);
        lv_label_set_text(entry, txt.c_str());
        lv_obj_set_style_text_color(entry, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(entry, &lv_font_montserrat_12, LV_PART_MAIN);
        lv_obj_align(entry, LV_ALIGN_TOP_LEFT, 10, y);
        y += 20;
    }

    // Back Button
    lv_obj_t* back_btn = lv_btn_create(settings_screen);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_size(back_btn, 80, 30);
    lv_obj_t* back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Back");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_btn, close_settings, LV_EVENT_CLICKED, NULL);
}