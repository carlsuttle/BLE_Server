// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: gmeter

#include "../ui.h"

void ui_watch_screen_init(void)
{
    ui_watch = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_watch, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_watchface = lv_img_create(ui_watch);
    lv_img_set_src(ui_watchface, &ui_img_watchface_png);
    lv_obj_set_width(ui_watchface, LV_SIZE_CONTENT);   /// 240
    lv_obj_set_height(ui_watchface, LV_SIZE_CONTENT);    /// 240
    lv_obj_set_align(ui_watchface, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_watchface, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_watchface, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_hourhand = lv_img_create(ui_watch);
    lv_img_set_src(ui_hourhand, &ui_img_clockwise_hour_png);
    lv_obj_set_width(ui_hourhand, LV_SIZE_CONTENT);   /// 18
    lv_obj_set_height(ui_hourhand, LV_SIZE_CONTENT);    /// 98
    lv_obj_set_x(ui_hourhand, 0);
    lv_obj_set_y(ui_hourhand, -28);
    lv_obj_set_align(ui_hourhand, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_hourhand, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_hourhand, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_pivot(ui_hourhand, 7, 61);

    ui_minhand = lv_img_create(ui_watch);
    lv_img_set_src(ui_minhand, &ui_img_clockwise_min_png);
    lv_obj_set_width(ui_minhand, LV_SIZE_CONTENT);   /// 18
    lv_obj_set_height(ui_minhand, LV_SIZE_CONTENT);    /// 99
    lv_obj_set_x(ui_minhand, 0);
    lv_obj_set_y(ui_minhand, -29);
    lv_obj_set_align(ui_minhand, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_minhand, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_minhand, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_pivot(ui_minhand, 9, 78);

    ui_sechand = lv_img_create(ui_watch);
    lv_img_set_src(ui_sechand, &ui_img_clockwise_sec_png);
    lv_obj_set_width(ui_sechand, LV_SIZE_CONTENT);   /// 31
    lv_obj_set_height(ui_sechand, LV_SIZE_CONTENT);    /// 180
    lv_obj_set_x(ui_sechand, 0);
    lv_obj_set_y(ui_sechand, -17);
    lv_obj_set_align(ui_sechand, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_sechand, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_sechand, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_pivot(ui_sechand, 9, 107);

}
